# Motors program for SCUTTLE running Jetson Nano, modified from RPi SCUTTLE
# Jetson Nano only has 2 PWM capable pins excluding the CPU fan, 
# so we use I2C to control an external PWM device (PCA9685)
# Code runs, but has not been tested on hardware
# Last update: Nov 10 2021 modified for Jetson Nano w/ PCA9685 - Carson F

import time             # Time keeping
import numpy as np      # Numpy for easy arrays
#import busio            # Provides I2C interface

#from board import SCL, SDA              # Import this board's I2C pin config
from L0_pca9685 import PCA9685           # Import PWM controller
import L1_i2c as i2c

# Configure I2C
# Q: L1_encoder uses I2C, but w/ smbus2. This uses busio, may be a problem since both need
#    to run together and use I2C. May need an L1_I2C to ensure I'm not duplicating I2C
#    -Carson
freq = 150
#i2c_bus = busio.I2C(SCL, SDA)
i2c_bus = i2c.get_i2c()

# Configure PCA9685
pca = PCA9685(i2c=i2c_bus)
pca.set_pwm_freq(1000)
pca.set_all_pwm(0, 1)

def computePWM(speed):              # take an argument in range [-1,1]
    if speed == 0:
        x = np.array([0,0])         # set all PWM to zero
    else:
        x = speed + 1.0              # change the range to [0,2]
        chA = 4096 * (0.5 * x)       # channel A sweeps low to high
        chB = 4096 * (1 - (0.5 * x)) # channel B sweeps high to low
        x = np.array([int(chA), int(chB)])     # store values to an array
    return(x)

def sendLeft(mySpeed):          # takes about 1 ms, slower than usual due to i2c control
    myPWM = computePWM(mySpeed)
    pca.set_pwm(0, 0, myPWM[0])
    pca.set_pwm(1, 0, myPWM[1])

def sendRight(mySpeed):         # takes about 1 ms
    myPWM = computePWM(mySpeed)
    pca.set_pwm(2, 0, myPWM[0])
    pca.set_pwm(3, 0, myPWM[1])

# THIS LOOP ONLY RUNS IF THE PROGRAM IS CALLED DIRECTLY
if __name__ == "__main__":
    try:
        while(1):
            print("motors.py: driving fwd")
            sendLeft(0.8)
            sendRight(0.8)
            time.sleep(4)                       # run fwd for 4 seconds
            print("motors.py: driving reverse")
            sendLeft(-0.8)
            sendRight(-0.8)
            time.sleep(4)                       # run reverse for 4 seconds
            sendLeft(0.8)
            sendRight(-0.8)
            time.sleep(4)  
            print("stopping motors 4 seconds")
            sendLeft(0)
            sendRight(0)
            time.sleep(4)
    except KeyboardInterrupt:
        sendLeft(0)
        sendRight(0)