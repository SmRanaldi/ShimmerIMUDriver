from shimmer import Shimmer
import time
import matplotlib.pyplot as plt
import numpy as np
import os

fs = 103
t = 5
inter_command_delay = 0.5
com_ports = ['Com5', 'Com8', 'Com9', 'Com12']
# com_ports = ['Com5']
# Com5 -> EA15
# Com8 -> D3CE
# Com3 -> EA24 (Check)
# Com9 -> E46C
# Com12 -> E91F

output_path = 'test_data'
if not os.path.exists(output_path):
    os.makedirs(output_path)

sensors_to_enable = ['SENSOR_MPU9150_GYRO','SENSOR_A_ACCEL']
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
    # for aa in data[s.com_port].values():
        # print(aa.shape)
    print(f"{data[s.com_port]['SENSOR_A_ACCEL'].shape[0]} samples for shimmer on {s.com_port}.")
    l.append(data[s.com_port]['SENSOR_A_ACCEL'].shape[0])
    plt.figure()
    for sens in sensors_to_enable:
        plt.plot(data[s.com_port][sens][:,0])
    plt.show()
min_l = np.min(l)

plt.figure()
for s in shimmers:
    plt.plot(data[s.com_port]['SENSOR_A_ACCEL'][-min_l:,0], label = s.com_port)
plt.legend()
plt.show()

for s in shimmers:
    s.disconnect()