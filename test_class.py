from shimmer import Shimmer
import time
import matplotlib.pyplot as plt

fs = 103
t = 5
inter_command_delay = 0.5
com_ports = ['Com3', 'Com5']

shimmers = [Shimmer(x, ['SENSOR_A_ACCEL'], fs) for x in com_ports]
for s in shimmers:
    s.connect()
    time.sleep(inter_command_delay)
for s in shimmers:
    s.initialize()
    time.sleep(inter_command_delay)
for s in shimmers:
    s.start()
print('Multi-shimmer acquisition started')
print('')
print('Press enter to stop acquisition...')
input('')
for s in shimmers:
    s.stop()
    time.sleep(inter_command_delay)
for s in shimmers:
    s.disconnect()
    time.sleep(inter_command_delay)
for s in shimmers:
    print(f"{s.data_structure['SENSOR_A_ACCEL'].shape[0]} samples, {fs*t} expected.")
    plt.figure()
    plt.plot(s.data_structure['SENSOR_A_ACCEL'])
    plt.show()

plt.figure()
for s in shimmers:
    plt.plot(s.data_structure['SENSOR_A_ACCEL'][:,0])
plt.show()