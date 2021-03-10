import sys
import struct
import serial
import numpy as np
import threading
import time
import pandas as pd

class Shimmer:

    # ----- Parameters -----
    baud_rate = 115200

    sensors_list = {
        # [0] - On command.
        # [1] - Data packet format
        # [2] - Order in payload (NOT SURE)

        'SENSOR_A_ACCEL': (0x80, 'HHH', 1),
        'SENSOR_MPU9150_GYRO': (0x040, '>hhh', 2),
        'SENSOR_LSM303DLHC_MAG': (0x20, '>hhh', 3),
        'SENSOR_GSR': (0x04, 'H', 4),
        'SENSOR_EXT_A7': (0x02, 'H', 5),
        'SENSOR_EXT_A6': (0x01, 'H', 6),
        'SENSOR_VBATT': (0x2000, 'H', 7),
        'SENSOR_D_ACCEL': (0x1000, 'hhh', 8),
        'SENSOR_EXT_A15': (0x0800, 'H', 9),
        'SENSOR_INT_A1': (0x0400, 'H', 10),
        'SENSOR_INT_A12': (0x0200, 'H', 11),
        'SENSOR_INT_A13': (0x0100, 'H', 12),
        'SENSOR_INT_A14': (0x800000, 'H', 13),
        'SENSOR_BMP180_PRESSURE': (0x40000, '>II', 14), # CHECK
        'SENSOR_EXG1_24BIT': (0x10, '>ii', 15),
        'SENSOR_EXG2_24BIT': (0x08, '>ii', 16),
        'SENSOR_EXG1_16BIT': (0x100000, '>hh', 17),
        'SENSOR_EXG2_16BIT': (0x080000, '>hh', 18),
        'SENSOR_BRIDGE_AMP': (0x8000, 'HH', 19),
    }

    commands = {
        'SET_SENSOR_COMMAND': 0x08,
        'SET_SAMPLING_RATE': 0x05,
        'START': 0x70,
        'STOP': 0x20,
    }

    # ----- Constructor -----
    def __init__(self, com_port, enabled_sensors, sampling_rate):
        self.com_port = com_port
        enabled_sensors = sorted(enabled_sensors, key = lambda x: self.sensors_list[x][2])
        self.enabled_sensors = enabled_sensors
        self.sampling_rate = sampling_rate
        self.data_structure = {sens: [] for sens in enabled_sensors}
        self.timestamp = []
        self.loop = threading.Thread(target = self.__core_loop__)
        self.is_running = 0
        self.is_ready = 0
        self.is_recording = 0
        self.memory = []

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
        self.connection = serial.Serial(self.com_port, self.baud_rate)
        self.connection.flushInput()
        print(f'Port opened for shimmer {self.com_port}')

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
            output_data[key] = np.array(value)
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
            for i in range(1,4):
                tmp_df = pd.DataFrame({sens+'_'+str(i): self.memory[sens][:,i-1]})
                output_dataframe = pd.concat([output_dataframe, tmp_df], axis=1, ignore_index=False)
                # output_dataframe[sens+'_'+str(i)] = self.memory[sens][:,i-1]
        output_dataframe.to_csv(filename, index=False)

    def disconnect(self):
        self.loop.join()
        self.connection.write(struct.pack('B', self.commands['STOP']))
        self.__wait_for_ack__()
        self.connection.close()