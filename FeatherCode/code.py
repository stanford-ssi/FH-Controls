import time
import board
import busio
from ulab import numpy as np
from helpers import *
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
dt = 0.1
state = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

print(sensor.calibration_status)
while not sensor.calibration_status == (3,3,0,3):
    print(sensor.calibration_status)



while True:
    angles = get_rotational_data(sensor)
    time.sleep(dt)
    print(angles)

    print("   ")