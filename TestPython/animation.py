import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from matplotlib.patches import Arrow

import numpy as np
from myobstacles import createmyobstacle
from myobstacles import plotrobotandobstacles


SIM_TIME = 10.  #total simulation time in (s)
TIMESTEP = 0.5  #time steps per calculation (Higher Value == larger steps between calculation) orig = 0.1(s) 
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)  
ROBOT_RADIUS = 0.5   #relative radius of the robot

position = np.array([0,0])
velocity = np.array([2,0])
obstacles = createmyobstacle(SIM_TIME, NUMBER_OF_TIMESTEPS, position, velocity)
plotrobotandobstacles(obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME)

