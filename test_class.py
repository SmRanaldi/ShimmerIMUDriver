import quaternion
from shimmer import Shimmer
import time
import matplotlib.pyplot as plt
import numpy as np
import os

fs = 102.4
t = 5
inter_command_delay = 0.5
com_ports = ['Com5', 'Com8', 'Com9', 'Com12']
com_ports = ['Com5']
# Com5 -> EA15
# Com8 -> D3CE
# Com3 -> EA24 (Check)
# Com9 -> E46C
# Com12 -> E91F

output_path = 'test_data'
if not os.path.exists(output_path):
    os.makedirs(output_path)

sensors_to_enable = ['SENSOR_MPU9150_GYRO','SENSOR_A_ACCEL', 'SENSOR_LSM303DLHC_MAG']
shimmers = [Shimmer(x, sensors_to_enable, fs) for x in com_ports]
print('Connecting...')
for s in shimmers:
    s.connect()
print('Initializing ...')
for s in shimmers:
    c = 0;
    while not s.is_running and c<10:
        s.initialize()
        c += 1
print('Starting...')
for s in shimmers:
    s.start()
print('Multi-shimmer acquisition started')
print('')
print('Press enter to stop acquisition...')
input('')
for s in shimmers:
    s.stop()

data = {}
l = []
for s in shimmers:
    data[s.com_port] = s.get_last_data()
    s.save_last_data(output_path + '/data_'+s.com_port+'.csv')
    s.save_quaternions(output_path+'/quaternions_'+s.com_port+'.csv')
    sampling_period = np.mean(np.diff(s.timestamp)/1000)
    print(f"Sampling period for Shimmer {s.com_port}: {sampling_period:2f}. Theoretical sampling period: {1/fs:2f}")
    print(f"{data[s.com_port]['SENSOR_A_ACCEL'].shape[0]} samples for shimmer on {s.com_port}.")
    l.append(data[s.com_port]['SENSOR_A_ACCEL'].shape[0])
    plt.subplot(121)
    for i in range(0,3):
        plt.plot(data[s.com_port]['SENSOR_MPU9150_GYRO'][:,i])
    plt.subplot(122)
    for i in range(0,3):
        plt.plot(data[s.com_port]['SENSOR_LSM303DLHC_MAG'][:,i])
    plt.show()
    error_value = np.max([np.mean(np.abs((data[s.com_port]['SENSOR_MPU9150_GYRO'][:np.min((200,l[-1])),i]))) for i in range(3)])
    print(f"Gyro error for Shimmer {s.com_port}: {error_value/(180/np.pi)} rad/s")
min_l = np.min(l)

plt.figure()
for s in shimmers:
    plt.plot(data[s.com_port]['SENSOR_A_ACCEL'], label = s.com_port)
plt.legend()
plt.show()

for s in shimmers:
    ang = s.get_euler_angles()
    ang *= 180/np.pi
    r_axis = s.get_axis_angle()
    print(r_axis.shape)
    fig = plt.figure()
    fig.add_subplot(121, projection='3d')
    # plt.plot(ang)
    plt.plot(r_axis[:,0],r_axis[:,1],r_axis[:,2],'ko')
    plt.plot(0,0,0,'ro',markersize=5)
    fig.add_subplot(122)
    plt.plot(r_axis[:,3]*(180/np.pi),'k')
    plt.show()
for s in shimmers:
    s.disconnect()