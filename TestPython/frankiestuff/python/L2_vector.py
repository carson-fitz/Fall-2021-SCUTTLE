# This program manipulates distance vectors in the robot coordinate frame,
# as well as arrays of vectors.  Pay attention to format of arguments since
# some functions are not yet optimized to handle numpy [1x2] vectors directly.
# Further functions will be added for rotation of vectors in various coordinate frames.
# Modified 11/20/21: Added cart2polar function, untested - Carson F

# Import external libraries
import numpy as np
import time

np.set_printoptions(precision=3)                    # after math operations, don't print long values

def polar2cart(r, alpha):                           # convert an individual vector to cartesian coordinates (in the robot frame)
    alpha = np.radians(alpha)                       # alpha*(np.pi/180) # convert to radians
    x = r * np.cos(alpha)                           # get x
    y = r * np.sin(alpha)                           # get y
    cart = np.round(np.array([x, y]), 3)            # vectorize and round
    return cart

def cart2polar_matrix(vel_target, heading):    # attempt to implement cart2polar using linear algebra, very likely does not work it's just for reference
    A = np.array([np.cos(heading), np.sin(heading), 0], 
                 [0,               0,               1])
    B = np.append(vel_target, heading)
    x = np.linalg.lstsq(A, B)
    return x

def cart2polar(target, heading, t=0.5):
    d_theta = np.arctan2(target[1], target[0]) - heading
    td = d_theta/t
    denom = (2*np.sin(np.abs(d_theta)/2))
    if denom==0:
        return np.array([0,0])
    rd = (td*np.sqrt(target[1]**2 + target[0]**2))/(2*np.sin(np.abs(d_theta)/2))
    return np.round(np.array([rd, td]), 3) 


def rotate(vec, theta):                             # describe a vector in global coordinates by rotating from body-fixed frame
    c, s = np.cos(theta), np.sin(theta)             # define cosines & sines
    R = np.array(((c, -s), (s, c)))                 # generate a rotation matrix
    vecGlobal = np.matmul(R, vec)                   # multiply the two matrices
    return vecGlobal


def sumVec(vec, loc):                               # add two vectors. (origin to robot, robot to obstacle)
    mySum = vec + loc                               # element-wise addition takes place
    return mySum                                    # return [x,y]
