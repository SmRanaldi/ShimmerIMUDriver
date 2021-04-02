import quaternion
from shimmer.base_class import ShimmerBaseClass
import time
import matplotlib.pyplot as plt
import numpy as np
import os
from visualize_3d_shimmer import visualize_3d

fs = 52.1
t = 5
inter_command_delay = 0.5
com_ports = ['Com5', 'Com8', 'Com9', 'Com12']
com_ports = ['Com9']
# Com5 -> EA15
# Com8 -> D3CE
# Com3 -> EA24 (Check)
# Com9 -> E46C
# Com12 -> E91F

# On linux, first bind the port:
# > sudo rfcomm N_PORT MAC_ADDRESS CHANNEL=1
# then update permissions
# > sudo chmod 666/dev/rfcomm{N_PORT}

output_path = 'test_data'
if not os.path.exists(output_path):
    os.makedirs(output_path)

sensors_to_enable = ['SENSOR_MPU9150_GYRO','SENSOR_A_ACCEL', 'SENSOR_LSM303DLHC_MAG']
shimmers = [ShimmerBaseClass(x, sensors_to_enable, fs) for x in com_ports]
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
    s.disconnect()

data = {}
l = []
for s in shimmers:
    data[s.com_port] = s.get_last_data()
    s.save_last_data(output_path + '/data_'+s.com_port.replace("/","")+'.csv')
    s.save_quaternions(output_path+'/quaternions_'+s.com_port.replace("/","")+'.csv')
    l.append(data[s.com_port]['SENSOR_A_ACCEL'].shape[0])

    sampling_period = np.mean(np.diff(s.timestamp)/1000)
    error_value = np.max([np.mean(np.abs((data[s.com_port]['SENSOR_MPU9150_GYRO'][:np.min((200,l[-1])),i]))) for i in range(3)])

    print(f"Sampling period for Shimmer {s.com_port}: {sampling_period:2f}. Theoretical sampling period: {1/fs:2f}")
    print(f"{data[s.com_port]['SENSOR_A_ACCEL'].shape[0]} samples for shimmer on {s.com_port}.")
    print(f"Gyro error for Shimmer {s.com_port}: {error_value/(180/np.pi)} rad/s")
    print("")
    plt.subplot(131)
    for i in range(0,3):
        plt.plot(data[s.com_port]['SENSOR_A_ACCEL'][:,i])
    plt.title("Accelerometer")
    plt.subplot(132)
    for i in range(0,3):
        plt.plot(data[s.com_port]['SENSOR_MPU9150_GYRO'][:,i])
    plt.title("Gyroscope")
    plt.subplot(133) 
    for i in range(0,3):
        plt.plot(data[s.com_port]['SENSOR_LSM303DLHC_MAG'][:,i])
    plt.title("Magnetometer")
    plt.show()
min_l = np.min(l)

for s in shimmers:
    ang = s.get_euler_angles()
    ang *= 180/np.pi
    r_axis = s.get_axis_angle()
    print(r_axis.shape)
    fig = plt.figure()
    ax = fig.add_subplot(131, projection='3d')
    # plt.plot(ang)
    x = np.arange(r_axis.shape[0])
    ax.scatter(r_axis[:,0],r_axis[:,1],r_axis[:,2],c=x,cmap=plt.cm.viridis)
    plt.plot(0,0,0,'ro',markersize=5)
    plt.title("Axis")
    ax = fig.add_subplot(132)
    ax.scatter(x, r_axis[:,3]*(180/np.pi),c=x,cmap=plt.cm.viridis)
    plt.title("Angle")
    fig.add_subplot(133)
    plt.plot(ang)
    plt.title("Euler angles")
    plt.show()
s = shimmers[0]

data = s.get_quaternions()
visualize_3d(data)