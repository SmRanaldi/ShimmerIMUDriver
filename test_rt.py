from shimmer.rt_orientation import RealTimeOrientation
import os

fs = 52.1
t = 5
inter_command_delay = 0.5
com_ports = ['Com5', 'Com8', 'Com9', 'com12']
com_ports = ['Com9']
# com5 -> ea15
# com8 -> d3ce
# com3 -> ea24 (check)
# com9 -> e46c
# com12 -> e91f

# on linux, first bind the port:
# > sudo rfcomm n_port mac_address channel=1
# then update permissions
# > sudo chmod 666/dev/rfcomm{n_port}

output_path = 'test_data'
if not os.path.exists(output_path):
    os.makedirs(output_path)

sensors_to_enable = ['sensor_mpu9150_gyro','sensor_a_accel', 'sensor_lsm303dlhc_mag']
shimmers = [RealTimeOrientation(x, fs, 20) for x in com_ports]
print('connecting...')
for s in shimmers:
    s.connect()
print('initializing ...')
for s in shimmers:
    c = 0
    while not s.is_running and c<10:
        s.initialize()
        c += 1
print('starting...')
for s in shimmers:
    s.start()
print('multi-shimmer acquisition started')
print('')
print('press enter to stop acquisition...')
input('')
for s in shimmers:
    s.stop()
    s.disconnect()