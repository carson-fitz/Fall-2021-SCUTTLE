# This program takes the encoder values from encoders, computes wheel movement
# and computes the movement of the wheelbase center based on SCUTTLE kinematics.
# This program runs on SCUTTLE with any CPU.
# Modified for Jetson 11/20/2021 Carson F

import L1_encoder as enc                    # local library for encoders
import numpy as np                          # library for math operations
from timeit import default_timer as timer
from time import sleep                                 

# define kinematics
R = 0.041                                   # wheel radius (meters)
L = 0.201                                   # half of wheelbase (meters)
res = (360/2**14)                           # resolution of the encoders (deg)
roll = int(360/res)                         # variable for rollover logic
gap = 0.5 * roll                            # degress specified as limit for rollover

A = np.array([[R/2, R/2], [-R/(2*L), R/(2*L)]])     # This matrix relates [PDL, PDR] to [XD,TD]

wait = 0.02                                 # wait time between encoder measurements (s)


def getTravel(deg0, deg1):                  # calculate the delta on Left wheel
    trav = deg1 - deg0                      # reset the travel reading
    if((-trav) >= gap):                     # if movement is large (has rollover)
        trav = (deg1 - deg0 + roll)         # forward rollover
    if(trav >= gap):
        trav = (deg1 - deg0 - roll)         # reverse rollover
    return(trav)


# Note:  this function takes at least 5ms to run
def getPdCurrent():
    encoders = enc.readShaftPositions()                   # grabs the current encoder readings in degrees
    degL0 = round(encoders[0], 1)           # reading in degrees.
    degR0 = round(encoders[1], 1)           # reading in degrees.
    t1 = timer()                            # timer() reports in seconds
    sleep(wait)                             # delay specified amount
    encoders = enc.read()                   # grabs the current encoder readings in degrees
    degL1 = round(encoders[0], 1)           # reading in degrees.
    degR1 = round(encoders[1], 1)           # reading in degrees.
    t2 = timer()                            # reading about .003 seconds
    global deltaT
    deltaT = round((t2 - t1), 3)            # new scalar dt value

    # ---- movement calculations
    travL = getTravel(degL0, degL1) * res   # grabs travel of left wheel, degrees
    travL = -1 * travL                      # this wheel is inverted from the right side
    travR = getTravel(degR0, degR1) * res   # grabs travel of right wheel, degrees

    # build an array of wheel speeds in rad/s
    travs = np.array([travL, travR])        # stores the travel in degrees
    travs = travs * 0.5                     # pulley ratio = 0.5 wheel turns per pulley turn
    travs = travs * 3.14 / 180              # convert degrees to radians
    travs = np.round(travs, decimals=3)     # round the array
    wheelSpeeds = travs / deltaT
    wheelSpeeds = np.round(wheelSpeeds, decimals=3)
    pdCurrents = wheelSpeeds                # store the updated most recent wheel speeds to the class variable
    return(pdCurrents)                      # returns [pdl, pdr] in radians/second


def getMotion():                            # this function returns the chassis speeds
    B = getPdCurrent()                      # store phidots to array B (here still in rad/s)
    C = np.matmul(A, B)                     # perform matrix multiplication
    C = np.round(C, decimals=3)             # round the matrix
    return(C)                               # returns a matrix containing [xDot, thetaDot]

# THIS SECTION ONLY RUNS IF THE PROGRAM IS CALLED DIRECTLY
if __name__ == "__main__":
    while True:
        C = getMotion()
        print("xdot(m/s), thetadot (rad/s):", C)
        sleep(0.1)