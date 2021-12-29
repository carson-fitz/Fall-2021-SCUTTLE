# This code runs on SCUTTLE with Jetson Nano setup
# Modified from RPi-SCUTTLE L1_encoder script - Carson F

# Import libraries
import L0_i2c as i2c        # i2c bus shared by several scripts
import numpy as np          # use numpy to build the angles array
import time                 # for keeping time

encL = 0x40         # encoder i2c address for LEFT motor
encR = 0x41         # encoder i2c address for RIGHT motor (this encoder has A1 pin pulled high)

def singleReading(encoderSelection):                                            # return a reading for an encoder in degrees (motor shaft angle)
    try:
        twoByteReading = i2c.get_i2c().read_i2c_block_data(encoderSelection, 0xFE, 2)     # request data from registers 0xFE & 0xFF of the encoder. Approx 700 microseconds.
        binaryPosition = (twoByteReading[0] << 6) | twoByteReading[1]           # remove unused bits 6 & 7 from byte 0xFF creating 14 bit value
        degreesPosition = binaryPosition*(360/2**14)                            # convert to degrees
        degreesAngle = round(degreesPosition,1)                                 # round to nearest 0.1 degrees
    except:
        print("Encoder reading failed.")  
        degreesAngle = 0
    return degreesAngle

def readShaftPositions():                                   # read both motor shafts.  approx 0.0023 seconds.
    try:
        rawAngle = singleReading(encL)                      # capture left motor shaft
        angle0 = 360.0 - rawAngle                           # invert the reading for left side only
        angle0 = round(angle0,1)                            # repeat rounding due to math effects
    except:
        print('Warning(I2C): Could not read left encoder')  # indicate which reading failed
        angle0 = 0
    try:
        rawAngle = singleReading(encR)                      # capture left motor shaft
        angle1 = 360.0 - rawAngle                           # invert the reading for left side only
        angle1 = round(angle1,1)                          
    except:
        print('Warning(I2C): Could not read right encoder') # indicate which reading failed
        angle1 = 0
    angles = np.array([angle0,angle1])
    return angles

# THIS LOOP RUNS IF THE PROGRAM IS CALLED DIRECTLY
if __name__ == "__main__":
    print("Testing Encoders")
    while True:
     encValues = readShaftPositions() # read the values.  Reading will only change if motor pulley moves
     # round the values and print them separated by a tab
     print("Left: ", encValues[0], "\t","Right: ", encValues[1])
     time.sleep(0.5)