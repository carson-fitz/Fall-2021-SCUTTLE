"""
Collision avoidance using Nonlinear Model-Predictive Control

author: Ashwin Bose (atb033@github.com)
"""
import objecttoalg as ob
import numpy as np
from scipy.optimize import minimize, Bounds
import time

SIM_TIME = 10.  #total simulation time in (s)
TIMESTEP = 0.5  #time steps per calculation (Higher Value == larger steps between calculation) orig = 0.1(s) 
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)  
ROBOT_RADIUS = 0.5   #relative radius of the robot
VMAX = 0.8   #max velocity of robot
VMIN = 0.1 #min velocity of robot

# collision cost parameters
Qc = 5.
kappa = 4.

# nmpc parameters
HORIZON_LENGTH = int(4) #number of prediction horizons
NMPC_TIMESTEP = 0.3  #time for each timestep of nmpc
upper_bound = [(1/np.sqrt(2)) * VMAX] * HORIZON_LENGTH * 2
lower_bound = [-(1/np.sqrt(2)) * VMAX] * HORIZON_LENGTH * 2


def simulate(myobstacles,robot_state, p_desired, robot_state_history):
    #obstacles = ob.createmyobstacle(SIM_TIME, NUMBER_OF_TIMESTEPS)  #generates obstacles given total simulation
                                                                 #and number of time steps
                                                                 #the obstacles are predefined in create_obstalces.py
                                                                 #and they can be modified if necessary
    #print("obs:", obstacles)                                                             
    #obstacles = np.array([[0],[0],[0],[0]] ) #hardcoded obstacle/velocity x,y,vx,vy
    obstacles = myobstacles

    obstacle_predictions = predict_obstacle_positions(obstacles)  #predicting obstacle position based n
                                                                            #Each obstacle defined as independent array of points with  
    xref = compute_xref(robot_state, p_desired,
                        HORIZON_LENGTH, NMPC_TIMESTEP) #position of MY robot
                                                           # Returns array of position(s) of different horizons
                                                          #cost function will chose the most optimal
    # compute velocity using nmpc (uses previous position-prediciton) and cost function
    vel, velocity_profile = compute_velocity(robot_state, obstacle_predictions, xref) #computes [Vx,Vy] of My robot based on optimal cost
    print('VelRobot:', vel)
    robot_state = update_state(robot_state, vel, TIMESTEP) #returns actual position of the robot after (Vel) applied 
    print('RobotNewPos:', robot_state)
    robot_state_history = robot_state

    #ob.plotrobotandobstacles(
    #    robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME)
    #function to print the robto to the screen based on previous states.
    return robot_state_history

def compute_velocity(robot_state, obstacle_predictions, xref):
    """
    Computes control velocity of the copter
    """
    #u0 = np.array([0] * 2 * HORIZON_LENGTH)
    u0 = np.random.rand(2*HORIZON_LENGTH)
    def cost_fn(u): return total_cost(
        u, robot_state, obstacle_predictions, xref)

    bounds = Bounds(lower_bound, upper_bound)

    res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds)
    velocity = res.x[:2]
    return velocity, res.x


def compute_xref(start, goal, number_of_steps, timestep):
    dir_vec = (goal - start)
    norm = np.linalg.norm(dir_vec)
    if norm < 0.1:
        new_goal = start
    else:
        dir_vec = dir_vec / norm
        new_goal = start + dir_vec * VMAX * timestep * number_of_steps
    return np.linspace(start, new_goal, number_of_steps).reshape((2*number_of_steps))


def total_cost(u, robot_state, obstacle_predictions, xref):
    x_robot = update_state(robot_state, u, NMPC_TIMESTEP)  #MY robot position
    c1 = tracking_cost(x_robot, xref)
    c2 = total_collision_cost(x_robot, obstacle_predictions)
    total = c1 + c2   #sun of tracking cost and total collision cost
    return total


def tracking_cost(x, xref):
    return np.linalg.norm(x-xref)


def total_collision_cost(robot, obstacles):
    total_cost = 0
    for i in range(HORIZON_LENGTH):
        for j in range(len(obstacles)):
            obstacle = obstacles[j]
            rob = robot[2 * i: 2 * i + 2]
            obs = obstacle[2 * i: 2 * i + 2]
            total_cost += collision_cost(rob, obs)
    return total_cost


def collision_cost(x0, x1):
    """
    Cost of collision between two robot_state
    """
    d = np.linalg.norm(x0 - x1)
    cost = Qc / (1 + np.exp(kappa * (d - 2*ROBOT_RADIUS)))
    return cost


def predict_obstacle_positions(obstacles):
    obstacle_predictions = []
    print("raw", obstacles)
    for i in range(np.shape(obstacles)[1]):
        obstacle = obstacles[:, i]
        obstacle_position = obstacle[:2]
        obstacle_vel = obstacle[2:]
        u = np.vstack([np.eye(2)] * HORIZON_LENGTH) @ obstacle_vel
        obstacle_prediction = update_state(obstacle_position, u, NMPC_TIMESTEP)
        print("obst prediction:", obstacle_prediction)
        obstacle_predictions.append(obstacle_prediction)
    return obstacle_predictions


def update_state(x0, u, timestep):
    """
    Computes the states of the system after applying a sequence of control signals u on
    initial state x0
    """
    N = int(len(u) / 2)
    lower_triangular_ones_matrix = np.tril(np.ones((N, N)))
    kron = np.kron(lower_triangular_ones_matrix, np.eye(2))

    new_state = np.vstack([np.eye(2)] * int(N)) @ x0 + kron @ u * timestep

    return new_state
