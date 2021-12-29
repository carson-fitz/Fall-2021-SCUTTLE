import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from matplotlib.patches import Arrow
import numpy as np
from numpy.core.numerictypes import obj2sctype

deadarray = []
def createmyobstacle(sim_time, num_timesteps, positionfromcam, velocityfromcam):

    # Example Obstacle 1
    #v = -2
    #p0 = np.array([5, 10])
    #obst = create_robot(p0, v, np.pi/2, sim_time,
    #                    num_timesteps).reshape(4, num_timesteps, 1) #4= number of predeicted horizons
    #obstacles = obst
    #return obstacles
    px = np.array(positionfromcam[0])
    py = np.array(positionfromcam[1])
    p0 = np.dstack([px,py])  
    obst = createrobot(p0, velocityfromcam, sim_time, num_timesteps)

    obstacles = obst

    return obstacles

def createrobot(p0, velocityfromcam, sim_time, num_timesteps):
    # Creates obstacles starting at p0 and moving at v in theta direction
    #t = np.linspace(0, sim_time, num_timesteps)
    #theta = theta * np.ones(np.shape(t))
    #vx = v * np.cos(theta)
    #vy = v * np.sin(theta)
    #v = np.stack([vx, vy])
    #p0 = p0.reshape((2, 1))
    #p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    #p = np.concatenate((p, v))
    #return p
    t = np.linspace(0, sim_time, num_timesteps)
    #theta = theta * np.ones(np.shape(t))
    velx = velocityfromcam[0] #Vx 
    vely = velocityfromcam[1] #Vy
    v = np.dstack([velx,vely])
    #print("vel:", v)
    p0 = p0.reshape((2,1))
    v = v.reshape((2,1))
    p = np.concatenate((p0,v))
    #p = p0 + np.cumsum(v, axis=0) * (sim_time / num_timesteps)
    #p = np.concatenate((p0, v))
    return p




def plotrobotandobstacles(robot, obstacles, robot_radius, num_steps, sim_time):
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, 10), ylim=(0, 10))
    ax.set_aspect('equal')
    ax.grid()
    line, = ax.plot([], [], '--r')
    robot_patch = Circle((robot[0], robot[1]), 
                         robot_radius, facecolor='green', edgecolor='black')
    obstacle_list = []
    for obstacle in range(np.shape(obstacles)[1]):
        obstacle = Circle((0, 0), robot_radius,
                          facecolor='aqua', edgecolor='black')
        obstacle_list.append(obstacle)
    #obstacle_list = obstacles
    def init():
        #ax.add_patch(robot_patch)
        for obstacle in obstacle_list:
            ax.add_patch(obstacle)
        line.set_data([], [])
        return obstacle_list

    def animate(i):
        #robot_patch.center = (robot[0, i], robot[1, i])
        for j in range(len(obstacle_list)):
            obstacle_list[j].center = (obstacles[0, j], obstacles[1, j])
        #line.set_data(robot[0, :i], robot[1, :i])
        return obstacle_list

    init()
    step = (sim_time / num_steps)
    for i in range(num_steps):
        animate(i)
        plt.pause(step)

    # Save animation
    #if not filename:
    #   return

    ani = animation.FuncAnimation(
        fig, animate, np.arange(1, num_steps), interval=200,
        blit=True, init_func=init)

    #ani.save(filename, "ffmpeg", fps=30)