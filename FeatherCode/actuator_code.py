import time
import board
import pwmio
from ulab import numpy as np

# LED setup for most CircuitPython boards:
led = pwmio.PWMOut(board.A1, duty_cycle=0, frequency=440, variable_frequency=True)
timecount = 0
timestep = 0.01
while True:
    print("running")
    timecount = timecount + timestep
    # PWM LED up and down
    led.duty_cycle = int(np.sin(timecount) * (65535/2) + (65535/2))
    print(np.sin(timecount))
    '''
    if i < 50:
        led.duty_cycle = int(i * 2 * 65535 / 100)  # Up
    else:
        led.duty_cycle = 65535 - int((i - 50) * 2 * 65535 / 100)  # Down
    '''
    time.sleep(0.1)