from shimmer import Shimmer
import time
import matplotlib.pyplot as plt
import numpy as np

fs = 103
t = 5
inter_command_delay = 0.5
com_ports = ['Com5', 'Com8', 'Com9', 'Com12']
# Com5 -> EA15
# Com8 -> D3CE
# Com3 -> EA24 (Check)
# Com9 -> E46C
# Com12 -> E91F

shimmers = [Shimmer(x, ['SENSOR_A_ACCEL'], fs) for x in com_ports]
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
for s in shimmers:
    s.disconnect()

data = {}
l = []
for s in shimmers:
    data[s.com_port] = s.get_stored_data()
    print(f"{data[s.com_port]['SENSOR_A_ACCEL'].shape[0]} samples for shimmer on {s.com_port}.")
    l.append(data[s.com_port]['SENSOR_A_ACCEL'].shape[0])
    plt.figure()
    plt.plot(data[s.com_port]['SENSOR_A_ACCEL'])
    plt.show()
min_l = np.min(l)

plt.figure()
for s in shimmers:
    plt.plot(data[s.com_port]['SENSOR_A_ACCEL'][-min_l:,0], label = s.com_port)
plt.legend()
plt.show()