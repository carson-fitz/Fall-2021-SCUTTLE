# Motors program for SCUTTLE running Jetson Nano, modified from RPi SCUTTLE
# Last update: Nov 20 2021 clean up after testing - Carson F

import time             # Time keeping
import numpy as np      # Numpy for easy arrays

from L0_pca9685 import PCA9685           # Import PWM controller
import L0_i2c as i2c

# Motor constants
l_channel =  np.array([0, 1])         # Channels 0 and 1 control left motor
r_channel = np.array([2, 3])         # Channels 2 and 3 control right motor

# Configure I2C
freq = 150

# Configure PCA9685
pca = PCA9685(i2c=i2c.get_i2c())
pca.set_pwm_freq(freq)
pca.set_all_pwm(0, 1)               # set all PWM to minimum duty cycle

def computePWM(speed):              # take an argument in range [-1,1]
    if speed == 0:
        x = np.array([0,0])         # set all PWM to zero
    else:
        x = speed + 1.0              # change the range to [0,2]
        chA = 4095 * (0.5 * x)       # channel A sweeps low to high
        chB = 4095 * (1 - (0.5 * x)) # channel B sweeps high to low
        x = np.array([int(chA), int(chB)])     # store values to an array
    return(x)

def sendLeft(mySpeed):          # takes about 1 ms, slower than before due to i2c control
    myPWM = computePWM(mySpeed)
    pca.set_pwm(l_channel[0], 0, myPWM[0])
    pca.set_pwm(l_channel[1], 0, myPWM[1])

def sendRight(mySpeed):         
    myPWM = computePWM(mySpeed)
    pca.set_pwm(r_channel[0], 0, myPWM[0])
    pca.set_pwm(r_channel[1], 0, myPWM[1])

def stop():
    sendLeft(0)
    sendRight(0)

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
            time.sleep(4)                       # turn for 4 seconds
            print("stopping motors 4 seconds")
            sendLeft(0)
            sendRight(0)
            time.sleep(4)                       # stop for 4 seconds
    except KeyboardInterrupt:                   # stop motors for keyboard interrupt
        stop()