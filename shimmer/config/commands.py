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

sensors_list = {
        # [0] - On command.
        # [1] - Data packet format
        # [2] - Order in payload (NOT SURE)
        # [3] - Number of channels
        # [4] - Calibration name (if present)

        'SENSOR_A_ACCEL': (0x80, 'HHH', 1, 3, 'A_ACCEL'), # Low-noise
        'SENSOR_MPU9150_GYRO': (0x40, '>hhh', 3, 3, 'GYRO'),
        'SENSOR_LSM303DLHC_MAG': (0x20, '>hhh', 4, 3, 'MAG'),
        'SENSOR_GSR': (0x04, 'H', 2, 1, ''),
        'SENSOR_EXT_A7': (0x02, 'H', 5, 1, ''),
        'SENSOR_EXT_A6': (0x01, 'H', 6, 1, ''),
        'SENSOR_VBATT': (0x2000, 'H', 7, 1, ''),
        'SENSOR_D_ACCEL': (0x1000, 'hhh', 8, 3, 'D_ACCEL'), # Wide range
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