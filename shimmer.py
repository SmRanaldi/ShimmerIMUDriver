import sys
import struct
import serial
import numpy as np
import threading
import time
import pandas as pd
import quaternion

class Shimmer:

    # ----- Parameters -----
    baud_rate = 115200
    length_calibration_packet = 21
    NORM_THS = 1e-6

    sensors_list = {
        # [0] - On command.
        # [1] - Data packet format
        # [2] - Order in payload (NOT SURE)
        # [3] - Number of channels
        # [4] - Calibration name (if present)

        'SENSOR_A_ACCEL': (0x80, 'HHH', 1, 3, 'A_ACCEL'),
        'SENSOR_MPU9150_GYRO': (0x040, '>hhh', 2, 3, 'GYRO'),
        'SENSOR_LSM303DLHC_MAG': (0x20, '>hhh', 3, 3, 'MAG'),
        'SENSOR_GSR': (0x04, 'H', 4, 1, ''),
        'SENSOR_EXT_A7': (0x02, 'H', 5, 1, ''),
        'SENSOR_EXT_A6': (0x01, 'H', 6, 1, ''),
        'SENSOR_VBATT': (0x2000, 'H', 7, 1, ''),
        'SENSOR_D_ACCEL': (0x1000, 'hhh', 8, 3, 'D_ACCEL'),
        'SENSOR_EXT_A15': (0x0800, 'H', 9, 1, ''),
        'SENSOR_INT_A1': (0x0400, 'H', 10, 1, ''),
        'SENSOR_INT_A12': (0x0200, 'H', 11, 1, ''),
        'SENSOR_INT_A13': (0x0100, 'H', 12, 1, ''),
        'SENSOR_INT_A14': (0x800000, 'H', 13, 1, ''),
        'SENSOR_BMP180_PRESSURE': (0x40000, '>II', 14, 2, ''), # CHECK
        'SENSOR_EXG1_24BIT': (0x10, '>ii', 15, 2, ''),
        'SENSOR_EXG2_24BIT': (0x08, '>ii', 16, 2, ''),
        'SENSOR_EXG1_16BIT': (0x100000, '>hh', 17, 2, ''),
        'SENSOR_EXG2_16BIT': (0x080000, '>hh', 18, 2, ''),
        'SENSOR_BRIDGE_AMP': (0x8000, 'HH', 19, 2, ''),
    }

    commands = {
        'SET_SENSOR_COMMAND': 0x08,
        'SET_SAMPLING_RATE': 0x05,
        'START': 0x70,
        'STOP': 0x20,
        'GET_A_ACCEL_CAL': 0x13,
        'GET_GYRO_CAL': 0x16,
        'GET_MAG_CAL': 0x19,
        'GET_D_ACCEL_CAL': 0x1C,
        'A_ACCEL_CAL_ACK': 0x12,
        'GYRO_CAL_ACK': 0x15,
        'MAG_CAL_ACK': 0x18,
        'D_ACCEL_CAL_ACK': 0x1B,
    }

    # ----- Constructor -----
    def __init__(self, com_port, enabled_sensors, sampling_rate):
        self.com_port = com_port
        self.quaternion_recording = 0
        if 'QUATERNION' in enabled_sensors:
            self.quaternion_recording = 1
            enabled_sensors = ['SENSOR_A_ACCEL', 'SENSOR_MPU9150_GYRO', 'SENSOR_LSM303DLHC_MAG']
        enabled_sensors = sorted(enabled_sensors, key = lambda x: self.sensors_list[x][2])
        self.enabled_sensors = [x for x in enabled_sensors if x in list(self.sensors_list.keys())]
        self.sampling_rate = sampling_rate
        self.data_structure = {sens: [] for sens in enabled_sensors}
        self.timestamp = []
        self.loop = threading.Thread(target = self.__core_loop__)
        self.is_running = 0
        self.is_ready = 0
        self.is_recording = 0
        self.memory = []
        self.calibration = {}
        if self.quaternion_recording:
            self.quaternion_output_data = []

    # ----- Private methods -----
    def __wait_for_ack__(self):
        ddata = ''
        ack = struct.pack('B', 0xff)
        while ddata != ack:
            ddata = self.connection.read(1)
        return

    def __core_loop__(self):
        next_byte = self.connection.read(1)
        while self.is_running:
            packettype = next_byte
            if packettype == struct.pack('B',0x00):
                ddata = bytearray()
                numbytes = len(ddata)
                ddata = self.connection.read(self.frame_length)

                ch_idx = 3
                if self.is_recording:
                    tmp_timestamp = list(struct.unpack('BBB', ddata[0:3]))
                    self.timestamp.append(tmp_timestamp[0] + 256*tmp_timestamp[1] + 65536*tmp_timestamp[2])
                    for sens in self.enabled_sensors:
                        buff_size = struct.calcsize(self.sensors_list[sens][1])
                        self.data_structure[sens].append(list(struct.unpack(self.sensors_list[sens][1],
                            ddata[ch_idx : ch_idx+ buff_size])))
                        ch_idx += buff_size

                next_byte = self.connection.read(1)
                if next_byte == struct.pack('B', 0xFF):
                    # Discard all the utility packets
                    while next_byte != struct.pack('B', 0x00):
                        next_byte = self.connection.read(1)

    # ----- Public methods -----
    def connect(self):
        try:
            self.connection = serial.Serial(self.com_port, self.baud_rate)
            self.connection.flushInput()
            print(f'Port opened for shimmer {self.com_port}')
        except:
            print(f'Shimmer on {self.com_port} not found.')

    def enable_sensors(self):
        self.is_ready = 0
        sensor_command = 0x00
        self.frame_length = 3
        for sens in self.enabled_sensors:
            sensor_command = sensor_command | self.sensors_list[sens][0]
            self.frame_length += struct.calcsize(self.sensors_list[sens][1])
        first_byte = sensor_command & 0xFF
        second_byte = (sensor_command & 0xFF00) >> 8
        third_byte = (sensor_command & 0xFF0000) >> 16
        self.connection.write(struct.pack('BBBB', self.commands['SET_SENSOR_COMMAND'], first_byte, second_byte, third_byte))
        self.__wait_for_ack__()

    def set_sampling_rate(self):
        self.is_ready = 0
        fs = round(32760/self.sampling_rate)
        fs = np.min([fs,65535])
        byte_low = fs&0xFF
        byte_high = (fs&0xFF00)>>8
        self.connection.write(struct.pack('BBB', 0x05, byte_low, byte_high))
        self.__wait_for_ack__()

    def set_accel_range(self):
        self.is_ready = 0
        # STILL TO ENCODE THE COMMAND
        self.connection.write(struct.pack('BBB', 0x4F, 0x00, 0x00))
        self.__wait_for_ack__()

    def initialize(self):
        self.enable_sensors()
        self.set_sampling_rate()
        self.set_accel_range()
        self.get_cal_parameters()
        self.connection.write(struct.pack('B', self.commands['START']))
        self.__wait_for_ack__()
        self.is_running = 1
        self.loop.start()
        print(f'Shimmer {self.com_port} ready.')

    def start(self):
        self.is_recording = 1
    
    def stop(self):
        self.is_recording = 0
        self.is_running = 0
    
    def get_last_data(self):
        output_data = {}
        for key, value in self.data_structure.items():
            print(key)
            output_data[key] = self.calibrate_channel(np.array(value), self.calibration[self.sensors_list[key][4]])
            # output_data[key] *= 100 # TESTTESTTEST
        self.memory = output_data
        return output_data

    def flush_data(self):
        self.timestamp = []
        for sens in self.enabled_sensors:
            self.data_structure[sens] = []

    def save_last_data(self, filename):
        if not self.memory:
            self.get_last_data()
        output_dataframe = pd.DataFrame([])
        output_dataframe['TIME'] = self.timestamp
        for sens in self.memory.keys():
            n_channels = self.sensors_list[sens][3]
            print(self.memory[sens].shape)
            for i in range(1,n_channels+1):
                tmp_df = pd.DataFrame({sens+'_'+str(i): self.memory[sens][:,i-1]})
                output_dataframe = pd.concat([output_dataframe, tmp_df], axis=1, ignore_index=False)
                # output_dataframe[sens+'_'+str(i)] = self.memory[sens][:,i-1]
        output_dataframe.to_csv(filename, index=False)

    def disconnect(self):
        self.loop.join()
        self.connection.write(struct.pack('B', self.commands['STOP']))
        self.__wait_for_ack__()
        self.connection.close()

    def get_cal_parameters(self):
        tmp_ack = []
        # LN accel
        self.connection.write(struct.pack('B', self.commands['GET_A_ACCEL_CAL']))
        self.__wait_for_ack__()
        ack_calibration = struct.pack('B', self.commands['A_ACCEL_CAL_ACK'])
        while tmp_ack != ack_calibration:
            tmp_ack = self.connection.read(1)
        ddata = bytearray()
        ddata = self.connection.read(self.length_calibration_packet)
        self.calibration['A_ACCEL'] = self.unpack_cal_parameters(ddata)
        # Gyroscope
        self.connection.write(struct.pack('B', self.commands['GET_GYRO_CAL']))
        self.__wait_for_ack__()
        ack_calibration = struct.pack('B', self.commands['GYRO_CAL_ACK'])
        while tmp_ack != ack_calibration:
            tmp_ack = self.connection.read(1)
        ddata = bytearray()
        ddata = self.connection.read(self.length_calibration_packet)
        self.calibration['GYRO'] = self.unpack_cal_parameters(ddata)
        # LN accel
        self.connection.write(struct.pack('B', self.commands['GET_MAG_CAL']))
        self.__wait_for_ack__()
        ack_calibration = struct.pack('B', self.commands['MAG_CAL_ACK'])
        while tmp_ack != ack_calibration:
            tmp_ack = self.connection.read(1)
        ddata = bytearray()
        ddata = self.connection.read(self.length_calibration_packet)
        self.calibration['MAG'] = self.unpack_cal_parameters(ddata)
        # LN accel
        self.connection.write(struct.pack('B', self.commands['GET_D_ACCEL_CAL']))
        self.__wait_for_ack__()
        ack_calibration = struct.pack('B', self.commands['D_ACCEL_CAL_ACK'])
        while tmp_ack != ack_calibration:
            tmp_ack = self.connection.read(1)
        ddata = bytearray()
        ddata = self.connection.read(self.length_calibration_packet)
        self.calibration['D_ACCEL'] = self.unpack_cal_parameters(ddata)

    def unpack_cal_parameters(self, ddata_in):
        # Bytes 0-5 -> 3 offset values (16 bit unsigned int, big endian)
        # Bytes 6-11 -> 3 sensitivity values (16 bit unsigned int, big endian)
        # Bytes 12-21 -> 9 elements of the alignment matrix (8 bit unsigned int)
        output_parameters = {}
        output_parameters['offset'] = np.array(struct.unpack('>HHH', ddata_in[0:6]))
        output_parameters['sensitivity'] = np.diag(np.array(struct.unpack('>HHH', ddata_in[6:12])))
        output_parameters['alignment'] = np.array(struct.unpack('bbbbbbbbb', ddata_in[12:])).reshape((3,3))/100
        return output_parameters
    
    def calibrate_channel(self, data_in, cal_parameters):
        # cal_data = inv(align) * inv(sensitivity) *(data - offset)
        if cal_parameters:
            data_out = np.zeros((1,data_in.shape[1]))
            for i in range(0,data_in.shape[0]):
                tmp = data_in[i,:].transpose()
                tmp = tmp - cal_parameters['offset']
                tmp = np.linalg.inv(cal_parameters['sensitivity'])@tmp
                tmp = np.linalg.inv(cal_parameters['alignment'])@tmp
                data_out = np.concatenate((data_out, tmp.transpose().reshape(1,-1)))
            return data_out[1:,:]
        else:
            return data_in
    
    def get_quaternions(self):

        if not self.memory:
            self.get_last_data()

        n_samples = len(self.timestamp)
        orientation_out = []

        acc = np.append(np.zeros((n_samples,1)), self.data_structure['SENSOR_A_ACCEL'], axis=1)
        gyr = np.append(np.zeros((n_samples,1)), self.data_structure['SENSOR_MPU9150_GYRO'], axis=1)
        mag = np.append(np.zeros((n_samples,1)), self.data_structure['SENSOR_LSM303DLHC_MAG'], axis=1)

        acc = quaternion.from_float_array(acc)
        gyr = quaternion.from_float_array(gyr)
        mag = quaternion.from_float_array(mag)

        tmp_orientation = np.quaternion(1,0,0,0) # Initialization
        for i in range(n_samples):
            tmp_orientation = self.get_orientation(acc[i], gyr[i], mag[i], tmp_orientation)
            orientation_out.append(tmp_orientation)

        return orientation_out

    def get_orientation(self, a, g, m, q):

        beta = 0.1
        sampling_period = 1/self.sampling_rate
        quaternion_norm = lambda x: np.sqrt(np.sum([c**2 for c in quaternion.as_float_array(x)])) 

        norm_a = quaternion_norm(a)
        if norm_a > self.NORM_THS: a /= norm_a
        norm_m = quaternion_norm(m)
        if norm_m > self.NORM_THS: a /= norm_m
        norm_q = quaternion_norm(q)
        if norm_q > self.NORM_THS: a /= norm_q

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
        fb = np.array([[2*BT[1]*(0.5-Q[2]**2 - Q[3]**2) + 2*BT[3]*(Q[1]*Q[3] - Q[0]*Q[2]) - M[1]],
                        [2*BT[1]*(Q[1]*Q[2] - Q[0]*Q[3]) + 2*BT[3]*(Q[0]*Q[1] + Q[2]*Q[3]) - M[2]],
                        [2*BT[1]*(Q[0]*Q[2] + Q[1]*Q[3]) + 2*BT[3]*(0.5 - Q[1]**2 - Q[2]**2) - M[3]]])

        # Nabla
        fgb = np.append(fg, fb, 0)
        Jgb = np.append(Jg, Jb, 0)
        f = Jgb.transpose()@fgb
        f = np.quaternion(*f)
        norm_f = quaternion_norm(f)
        if norm_f > self.NORM_THS: f /= norm_f

        # Eq. 30 in the paper
        q_dot = 0.5 * q * g - beta*f
        q_next = q + sampling_period*q_dot
        norm_q_next = quaternion_norm(q_next)
        q_next /= norm_q_next
        
        return q_next

    def save_quaternions(self, filename):
        quat = pd.DataFrame(quaternion.as_float_array(self.get_quaternions()))
        quat.to_csv(filename)

    def get_euler_angles(self):
        quat = self.get_quaternions()
        angles_out = []
        for q in quat:
            angles_out.append(quaternion.as_euler_angles(q))
        return np.array(angles_out)

    def get_axis_angle(self):
        quat = self.get_quaternions()
        axis_out = []
        angle_out = []
        for q in quat:
            Q = quaternion.as_float_array(q)
            row = []
            axis_out.append(Q[1:]/np.sqrt(1-Q[0]**2))
            angle_out.append(2*np.arccos(Q[0]))
        angle_out = np.array(angle_out)
        axis_out = np.array(axis_out)
        return np.concatenate((axis_out,angle_out.reshape(-1,1)), axis=1)