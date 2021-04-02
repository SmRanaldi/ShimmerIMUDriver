from shimmer.base_class import ShimmerBaseClass
import numpy as np
import pandas as pd
import quaternion
import json
import os

class ShimmerQuaternion(ShimmerBaseClass):

    def __init__(self, com_port, sampling_rate):

        enabled_sensors = ['SENSOR_A_ACCEL', 'SENSOR_MPU9150_GYRO', 'SENSOR_LSM303DLHC_MAG']
        super().__init__(com_port, enabled_sensors, sampling_rate)
        self.quaternions = []
        self.delta_t = 1./self.sampling_rate

# ----- Core function override -----

    def __processing_function__(self):
        acc = self.calibrate_channel(np.array([self.data_structure['SENSOR_A_ACCEL'][-1]]), self.calibration[self.sensors_list['SENSOR_A_ACCEL'][4]])
        gyr = self.calibrate_channel(np.array([self.data_structure['SENSOR_MPU9150_GYRO'][-1]]), self.calibration[self.sensors_list['SENSOR_MPU9150_GYRO'][4]])
        mag = self.calibrate_channel(np.array([self.data_structure['SENSOR_LSM303DLHC_MAG'][-1]]), self.calibration[self.sensors_list['SENSOR_LSM303DLHC_MAG'][4]])
        acc = quaternion.from_float_array(np.append(np.zeros((1,1)), acc, axis=1))
        gyr = quaternion.from_float_array(np.append(np.zeros((1,1)), gyr, axis=1))
        mag = quaternion.from_float_array(np.append(np.zeros((1,1)), mag, axis=1))

        if self.quaternions:
            tmp_orientation = self.get_orientation(acc[0], gyr[0], mag[0], self.quaternions[-1], self.delta_t)
        else:
            tmp_orientation = self.get_orientation(acc[0], gyr[0], mag[0], np.quaternion(1,0,0,0), self.delta_t)

        self.quaternions.append(tmp_orientation.copy())

# ----- Public methods -----

    def get_orientation(self, a, g, m, q, dt):

        # beta = 0.03
        quaternion_norm = lambda x: np.sqrt(np.sum([c**2 for c in quaternion.as_float_array(x)])) 

        norm_a = quaternion_norm(a)
        if norm_a > self.NORM_THS: a = a / norm_a
        norm_m = quaternion_norm(m)
        if norm_m > self.NORM_THS: m = m / norm_m
        # norm_q = quaternion_norm(q)
        # if norm_q > self.NORM_THS: a /= norm_q

        g /= (180/np.pi) # Get gyro data in radians/s

        ht = quaternion.as_float_array(q * m * q.conj())
        bt = np.quaternion(0, np.sqrt(ht[1]**2 + ht[2]**2), 0, ht[3])
        BT = quaternion.as_float_array(bt)
        Q = quaternion.as_float_array(q)
        M = quaternion.as_float_array(m)
        A = quaternion.as_float_array(a)

        # Arrays for gradient descent. If there's a bug in the code, it 
        # is probably here.
        Jb = np.array([[-2*BT[3]*Q[2], 2*BT[3]*Q[3], -4*BT[1]*Q[2] - 2*BT[3]*Q[0], -4*BT[1]*Q[3] + 2*BT[3]*Q[1]],
                        [-2*BT[1]*Q[3] + 2*BT[3]*Q[1], 2*BT[1]*Q[2] + 2*BT[3]*Q[0], 2*BT[1]*Q[1] + 2*BT[3]*Q[3], -2*BT[1]*Q[0] + 2*BT[3]*Q[2]],
                        [2*BT[1]*Q[2], 2*BT[1]*Q[3] - 4*BT[3]*Q[1], 2*BT[1]*Q[0] - 4*BT[3]*Q[2], 2*BT[1]*Q[1]]])
        Jg = np.array([[-2*Q[2], 2*Q[3], -2*Q[0], 2*Q[1]],
                        [2*Q[1], 2*Q[0], 2*Q[3], 2*Q[2]],
                        [0, -4*Q[1], -4*Q[2], 0]])
        fg = np.array([[2*(Q[1]*Q[3] - Q[0]*Q[2]) - A[1]],
                       [2*(Q[0]*Q[1] + Q[2]*Q[3]) - A[2]],
                       [2*(0.5 - Q[1]**2 - Q[2]**2) - A[3]]])
        fb = np.array([[2*BT[1]*(0.5 - Q[2]**2 - Q[3]**2) + 2*BT[3]*(Q[1]*Q[3] - Q[0]*Q[2]) - M[1]],
                        [2*BT[1]*(Q[1]*Q[2] - Q[0]*Q[3]) + 2*BT[3]*(Q[0]*Q[1] + Q[2]*Q[3]) - M[2]],
                        [2*BT[1]*(Q[0]*Q[2] + Q[1]*Q[3]) + 2*BT[3]*(0.5 - Q[1]**2 - Q[2]**2) - M[3]]])

        # Nabla
        fgb = np.append(fg, fb, 0) # 6x1 matrix
        Jgb = np.append(Jg, Jb, 0) # 6x4 matrix
        f = Jgb.transpose()@fgb
        f = np.quaternion(*f)
        norm_f = quaternion_norm(f)
        f = f / norm_f
        # if norm_f > self.NORM_THS: f /= norm_f

        # Eq. 30 in the paper
        q_dot = 0.5 * q * g - self.BETA*f
        q_next = q + dt*q_dot
        norm_q_next = quaternion_norm(q_next)
        q_next = q_next / norm_q_next
        
        return q_next

    def save_quaternions(self, filename):
        quat = pd.DataFrame(quaternion.as_float_array(self.quaternions))
        quat.to_csv(filename)

    def get_euler_angles(self):
        quat = self.quaternions
        angles_out = []
        angles_out = [quaternion.as_euler_angles(q) for q in quat]
        return np.array(angles_out)

    def get_axis_angle(self):
        quat = self.quaternions
        axis_out = []
        angle_out = []
        for q in quat:
            Q = quaternion.as_float_array(q)
            row = []
            axis_out.append((Q[1:]/np.sqrt(1-Q[0]**2)).copy())
            angle_out.append((2*np.arccos(Q[0])).copy())
        angle_out = np.array(angle_out)
        axis_out = np.array(axis_out)
        return np.concatenate((axis_out,angle_out.reshape(-1,1)), axis=1)