from shimmer import Shimmer
import time

s = Shimmer('Com3',['SENSOR_A_ACCEL'], 102.4)
s.connect()
s.initialize()
s.start()
time.sleep(5)
s.stop()