import sys, struct, serial
import numpy as np
import matplotlib.pyplot as plt

def wait_for_ack():
    ddata = ''
    ack = struct.pack('B', 0xff)
    while ddata != ack:
        ddata = ser.read(1)
    return
def set_sampling_rate_command(fs):
    fs = round(32760/fs)
    fs = np.min([fs,65535])
    byte_low = fs&0xFF
    byte_high = (fs&0xFF00)>>8
    return struct.pack('BBB', 0x05, byte_low, byte_high)
	
sensorsDict = {
	'SENSOR_A_ACCEL': 0x80,
	'SENSOR_MPU9150_GYRO': 0x040,
	'SENSOR_LSM303DLHC_MAG': 0x20,
	'SENSOR_GSR': 0x04,
	'SENSOR_EXT_A7': 0x02,
	'SENSOR_EXT_A6': 0x01,
	'SENSOR_VBATT': 0x2000,
	'SENSOR_D_ACCEL': 0x1000,
	'SENSOR_EXT_A15': 0x0800,
	'SENSOR_INT_A1': 0x0400,
	'SENSOR_INT_A12': 0x0200,
	'SENSOR_INT_A13': 0x0100,
	'SENSOR_INT_A14': 0x800000,
	'SENSOR_BMP180_PRESSURE': 0x40000,
	'SENSOR_EXG1_24BIT': 0x10,
	'SENSOR_EXG2_24BIT': 0x08,
	'SENSOR_EXG1_16BIT': 0x100000,
	'SENSOR_EXG2_16BIT': 0x080000,
	'SENSOR_BRIDGE_AMP': 0x8000
}
		
enabledSensors = ['SENSOR_A_ACCEL', 'SENSOR_LSM303DLHC_MAG']

if len(sys.argv) < 2:
    print('Error, specify the COM port.')
else:
    ser = serial.Serial(sys.argv[1], 115200)
    ser.flushInput()
    print('Port opened')
    # Enable accelerometer
    # 0x08 -> SET_SENSORS_COMMAND
    sensor_command = 0x00
    for sens in enabledSensors:
        sensor_command = sensor_command | sensorsDict[sens]
    first_byte = sensor_command & 0xFF
    second_byte = (sensor_command & 0xFF00) >> 8
    third_byte = (sensor_command & 0xFF0000) >> 16
    ser.write(struct.pack('BBBB', 0x08, first_byte, second_byte, third_byte))
    wait_for_ack()
    print('Sensors enabled')
    # Set sampling rate
    # 0x05 -> SET_SAMPLING_RATE_COMMAND
    ser.write(set_sampling_rate_command(102.4))
    wait_for_ack()
    print('Sampling rate set')
    # Set accel range
    ser.write(struct.pack('BBB', 0x4F, 0x00, 0x00))
    wait_for_ack()
    print('Range set')
    # Get buffer size
    ser.write(struct.pack('B', 0x36))
    wait_for_ack()
    response = ''
    while response != struct.pack('B', 0x35):
        response = ser.read(1)
    response = struct.unpack('B', ser.read(1))
    print(response)
    #Send START
    ser.write(struct.pack('B', 0x70))
    wait_for_ack()
    print('Recording')

    ddata = bytearray() 
    numbytes = 0
    framesize =  15
    timestamp_tot = []
    accel_tot = []
    accel2_tot = []
    accel = []
    accel2 = []
    timestamp = []
    next_byte = ser.read(1)
    try:
        while True:
            packettype = next_byte
            if packettype == struct.pack('B',0x00):
                ddata = bytearray()
                numbytes = len(ddata)
                # while numbytes < framesize:
                #     ddata += ser.read(1)
                #     numbytes = len(ddata)
                ddata = ser.read(framesize)

                # packettype = struct.unpack('B', data[0:1])
                timestamp = struct.unpack('BBB', ddata[0:3])
                accel = struct.unpack('HHH', ddata[3:9])
                # accel = struct.unpack('>hhh', ddata[3:9])
                accel2 = struct.unpack('>hhh', ddata[9:framesize])

                timestamp_tot.append(timestamp[0] + 256*timestamp[1] + 65536*timestamp[2])
                accel_tot.append([accel[0], accel[1], accel[2]])
                accel2_tot.append([accel2[0], accel2[1], accel2[2]])
                next_byte = ser.read(1)
                if next_byte == struct.pack('B', 0xFF):
                    # Discard all the utility packets
                    print('!')
                    while next_byte != struct.pack('B', 0x00):
                        next_byte = ser.read(1)

    except KeyboardInterrupt:
        # Stop the shimmer
        ser.write(struct.pack('B', 0x20))
        wait_for_ack()
        print('Shimmer stopped')
        ser.close()
        accel_tot = np.array(accel_tot)
        plt.figure()
        plt.subplot(121)
        plt.plot(accel_tot)
        plt.subplot(122)
        plt.plot(accel2_tot)
        plt.show()
        print(np.mean(timestamp_tot))