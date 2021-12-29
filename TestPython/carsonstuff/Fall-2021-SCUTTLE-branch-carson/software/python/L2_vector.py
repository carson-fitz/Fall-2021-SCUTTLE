# This program manipulates distance vectors in the robot coordinate frame,
# as well as arrays of vectors.  Pay attention to format of arguments since
# some functions are not yet optimized to handle numpy [1x2] vectors directly.
# Further functions will be added for rotation of vectors in various coordinate frames.
# Modified 11/20/21: Added cart2polar function, untested - Carson F

# Import external libraries
import numpy as np
import time
from math import atan2

np.set_printoptions(precision=3)                    # after math operations, don't print long values
displacement = np.zeros(2)

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
    #global displacement
    t_heading = atan2(target[0], target[1])
    d_theta = t_heading - heading
    print("dtheta",d_theta)
    if (d_theta < 0.0001) & (d_theta > -0.0001):
        #displacement += t*np.array([target[1]/t, 0])
        return np.array([np.sqrt(target[1]**2 + target[0]**2)/t, 0])
    
    td = d_theta/t    
    #rd = (td*np.sqrt(target[1]**2 + target[0]**2))/(2*np.sin(np.abs(d_theta)/2))
    rd = ((np.sqrt(target[1]**2 + target[0]**2)/(2*np.sin(np.abs(d_theta/2))))*np.abs(d_theta)) / t
    motion = np.array([rd, td])
    #displacement += motion * t
    #print("displacement ", displacement)
    print("here")
    return motion


def rotate(vec, theta):                             # describe a vector in global coordinates by rotating from body-fixed frame
    c, s = np.cos(theta), np.sin(theta)             # define cosines & sines
    R = np.array(((c, s), (s, -c)))                 # generate a rotation matrix
    vecGlobal = np.matmul(R, vec)                   # multiply the two matrices
    return vecGlobal


def sumVec(vec, loc):                               # add two vectors. (origin to robot, robot to obstacle)
    mySum = vec + loc                               # element-wise addition takes place
    return mySum                                    # return [x,y]
