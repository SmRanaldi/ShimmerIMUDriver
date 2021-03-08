import sys
import struct
import serial
import numpy as np
import threading
import time

class Shimmer:

    # ----- Parameters -----
    baud_rate = 115200

    sensors_list = {
        # [0] - On command.
        # [1] - Data packet format

        'SENSOR_A_ACCEL': (0x80, 'HHH'),
        'SENSOR_MPU9150_GYRO': (0x040, '>hhh'),
        'SENSOR_LSM303DLHC_MAG': (0x20, '>hhh'),
        'SENSOR_GSR': (0x04, 'H'),
        'SENSOR_EXT_A7': (0x02, 'H'),
        'SENSOR_EXT_A6': (0x01, 'H'),
        'SENSOR_VBATT': (0x2000, 'H'),
        'SENSOR_D_ACCEL': (0x1000, 'hhh'),
        'SENSOR_EXT_A15': (0x0800, 'H'),
        'SENSOR_INT_A1': (0x0400, 'H'),
        'SENSOR_INT_A12': (0x0200, 'H'),
        'SENSOR_INT_A13': (0x0100, 'H'),
        'SENSOR_INT_A14': (0x800000, 'H'),
        'SENSOR_BMP180_PRESSURE': (0x40000, '>II'), # CHECK
        'SENSOR_EXG1_24BIT': (0x10, '>ii'),
        'SENSOR_EXG2_24BIT': (0x08, '>ii'),
        'SENSOR_EXG1_16BIT': (0x100000, '>hh'),
        'SENSOR_EXG2_16BIT': (0x080000, '>hh'),
        'SENSOR_BRIDGE_AMP': (0x8000, 'HH'),
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
        self.enabled_sensors = enabled_sensors
        self.sampling_rate = sampling_rate
        self.data_structure = {sens: [] for sens in enabled_sensors}
        self.timestamp = []
        self.loop = threading.Thread(target = self.__core_loop__)
        self.is_running = 0

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

                self.timestamp.append(list(struct.unpack('BBB', ddata[0:3])))
                ch_idx = 3
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
        print('Sensors enabled')

    def set_sampling_rate(self):
        fs = round(32760/self.sampling_rate)
        fs = np.min([fs,65535])
        byte_low = fs&0xFF
        byte_high = (fs&0xFF00)>>8
        self.connection.write(struct.pack('BBB', 0x05, byte_low, byte_high))
        self.__wait_for_ack__()
        print('Sampling rate set')

    def set_accel_range(self):
        # STILL TO ENCODE THE COMMAND
        self.connection.write(struct.pack('BBB', 0x4F, 0x00, 0x00))
        self.__wait_for_ack__()
        print('Range set')

    def initialize(self):
        self.enable_sensors()
        self.set_sampling_rate()
        self.set_accel_range()
        self.connection.write(struct.pack('B', self.commands['START']))
        self.__wait_for_ack__()
        self.loop.start()

    def start(self):
        print('Recording')
        self.flush_data()
        self.is_running = 1
    
    def stop(self):
        self.is_running = 0
        time.sleep(0.5)
        print('Acquisition stopped.')
        for key, value in self.data_structure.items():
            self.data_structure[key] = np.array(value)

    def flush_data(self):
        for sens in self.enabled_sensors:
            self.data_structure[sens] = []

    def disconnect(self):
        self.loop.join()
        self.connection.write(struct.pack('B', self.commands['STOP']))
        self.__wait_for_ack__()
        self.connection.close()
