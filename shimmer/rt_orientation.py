from shimmer.shimmer_quaternion import ShimmerQuaternion
import numpy as np
import pandas as pd
import quaternion
import json
import os
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

class RealTimeOrientation(ShimmerQuaternion):

    def __init__(self, com_port, sampling_rate, buffer_size):

        self.buffer_size = buffer_size
        self.buffer_counter = 0
        super().__init__(com_port, sampling_rate)
        self.quaternions = []
        self.delta_t = 1./self.sampling_rate

        # ---- Initialize figure ----
        self.vertices = np.array([[1.5,1.,0.],[1.5,1.,0.3],
                            [1.5,-1.,0.],[1.5,-1.,0.3],
                            [-1.5,-1.,0.],[-1.5,-1.,0.3],
                            [-1.5,1.,0.],[-1.5,1.,0.3]])
        self.points = self.vertices

        plt.ion()
        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)
        self.ax.set_xlim3d([-2,2])
        self.ax.set_ylim3d([-2,2])
        self.ax.set_zlim3d([-2,2])
        self.line = self.ax.scatter([],[],[], s=40, color='k')

        # plt.show()

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

        tmp_vert = np.ones(self.vertices.shape)
        for j in range(self.vertices.shape[0]):
            tmp_quaternion = np.insert(np.array(self.vertices[j,:]),0,0.)
            tmp_quaternion = np.quaternion(*tmp_quaternion)
            tmp_quaternion = tmp_orientation * tmp_quaternion * tmp_orientation.conj()
            tmp_vert[j,:] = quaternion.as_float_array(tmp_quaternion)[1:]
        self.points = tmp_vert

        self.buffer_counter += 1
        if self.buffer_counter == self.buffer_size:
            self.line._offsets3d=[self.points[:,0],self.points[:,1],self.points[:,2]]
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        self.buffer_counter %= self.buffer_size 

        self.quaternions.append(tmp_orientation)


