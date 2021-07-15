import sys
import struct
import serial
import numpy as np
import threading
import pandas as pd
import quaternion
import json
import os


BASE_PATH = os.path.split(os.path.abspath(__file__))[0]
CONFIG_PATH = os.path.join(BASE_PATH,"config")

class ShimmerBaseClass:

    # ----- Parameters -----
    baud_rate = 115200
    length_calibration_packet = 21
    NORM_THS = 1e-10
    BETA = np.sqrt(3/4) * 0.003# Equation 33 in the paper

    with open(os.path.join(CONFIG_PATH,"sensors_list.json"),'r') as f:
        sensors_list = json.load(f)

    # # Order of the parameters for each sensor in sensors_list
    # # [0] - ON COMMAND.
    # # [1] - DATA PACKET FORMAT
    # # [2] - ORDER IN PAYLOAD (NOT SURE)
    # # [3] - NUMBER OF CHANNELS
    # # [4] - CALIBRATION NAME (IF PRESENT)
    
    with open(os.path.join(CONFIG_PATH,"commands.json"),'r') as f:
        commands = json.load(f)

    # ----- Constructor -----
    def __init__(self, com_port, enabled_sensors, sampling_rate):
        self.com_port = com_port
        self.quaternion_recording = 0
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
                    self.timestamp.append(float((tmp_timestamp[0] + 256*tmp_timestamp[1] + 65536*tmp_timestamp[2])/32768 * 1000)) # In ms, from the C# API
                    for sens in self.enabled_sensors:
                        buff_size = struct.calcsize(self.sensors_list[sens][1])
                        self.data_structure[sens].append(list(struct.unpack(self.sensors_list[sens][1],
                            ddata[ch_idx : ch_idx+ buff_size])))
                        ch_idx += buff_size

                    self.__processing_function__()

                next_byte = self.connection.read(1)
                if next_byte == struct.pack('B', 0xFF):
                    # Discard all the utility packets
                    while next_byte != struct.pack('B', 0x00):
                        next_byte = self.connection.read(1)

    def __processing_function__(self):
        pass

    # ----- Public methods -----
    def connect(self):
        try:
            self.connection = serial.Serial(self.com_port, self.baud_rate, timeout=10) # Check here, timeout must be larger for Windows systems
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
        # fs = 1024/self.sampling_rate # From the Shimmer C# API, need to check
        # fs = int(fs)
        fs = int(32768/self.sampling_rate) # From the Shimmer C# API
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
            output_data[key] = self.calibrate_channel(np.array(value), self.calibration[self.sensors_list[key][4]])
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
        self.calibration['GYRO']['sensitivity'] = self.calibration['GYRO']['sensitivity']/100

        # Magnetometer
        self.connection.write(struct.pack('B', self.commands['GET_MAG_CAL']))
        self.__wait_for_ack__()
        ack_calibration = struct.pack('B', self.commands['MAG_CAL_ACK'])
        while tmp_ack != ack_calibration:
            tmp_ack = self.connection.read(1)
        ddata = bytearray()
        ddata = self.connection.read(self.length_calibration_packet)
        self.calibration['MAG'] = self.unpack_cal_parameters(ddata)
        # self.calibration['MAG']['alignment'] = self.calibration['MAG']['alignment'][[2,1,0],:] # Test swap channels

        # WR Accel
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
        output_parameters['offset'] = np.array(struct.unpack('>hhh', ddata_in[0:6]))
        output_parameters['sensitivity'] = np.diag(np.array(struct.unpack('>hhh', ddata_in[6:12])))
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