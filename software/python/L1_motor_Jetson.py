# Motors program for SCUTTLE running Jetson Nano, modified from RPi SCUTTLE
# Jetson Nano only has 2 PWM capable pins excluding the CPU fan, 
# so we use I2C to control an external PWM device (PCA9685)
# Code runs, but has not been tested on hardware
# Last update: Nov 10 2021 modified for Jetson Nano w/ PCA9685 - Carson F

import time             # Time keeping
import numpy as np      # Numpy for easy arrays
import busio            # Provides I2C interface

from board import SCL, SDA              # Import this board's I2C pin config
from adafruit_pca9685 import PCA9685    # Import PWM controller

# Configure I2C and PCA9685
freq = 150
i2c_bus = busio.I2C(SCL, SDA)
print(int(SCL))
print(int(SDA))
pca = PCA9685(i2c_bus)
pca.frequency = freq
pca.channels[0].duty_cycle = 0
pca.channels[1].duty_cycle = 0

def computePWM(speed):              # take an argument in range [-1,1]
    if speed == 0:
        x = np.array([0,0])         # set all PWM to zero
    else:
        x = speed + 1.0              # change the range to [0,2]
        chA = 4096 * (0.5 * x)       # channel A sweeps low to high
        chB = 4096 * (1 - (0.5 * x)) # channel B sweeps high to low
        x = np.array([int(chA), int(chB)])     # store values to an array
    return(x)

def sendLeft(mySpeed):          # takes at least 0.3 ms
    myPWM = computePWM(mySpeed)
    pca.channels[0].duty_cycle = myPWM[0]
    pca.channels[1].duty_cycle = myPWM[1]

#def sendRight(mySpeed):         # takes at least 0.3 ms
#    myPWM = computePWM(mySpeed)
#    right_chB.ChangeDutyCycle(myPWM[0]*100)
#    right_chA.ChangeDutyCycle(myPWM[1]*100)

# THIS LOOP ONLY RUNS IF THE PROGRAM IS CALLED DIRECTLY
if __name__ == "__main__":
    init()
    try:
        while(1):
            print("motors.py: driving fwd")
            sendLeft(0.8)
            #sendRight(0.8)
            time.sleep(4)                       # run fwd for 4 seconds
            print("motors.py: driving reverse")
            sendLeft(-0.8)
            #sendRight(-0.8)
            time.sleep(4)                       # run reverse for 4 seconds
            print("stopping motors 4 seconds")
            sendLeft(0)
            #sendRight(0)
            time.sleep(4)
    except:
        pca.deinit()
