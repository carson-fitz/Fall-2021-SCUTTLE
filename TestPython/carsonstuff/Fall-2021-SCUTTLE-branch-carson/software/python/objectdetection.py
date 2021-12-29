from logging import error
from re import I
from typing import TYPE_CHECKING
from warnings import catch_warnings
from matplotlib import animation
from numpy.lib.function_base import piecewise
from numpy.lib.type_check import mintypecode
import pyzed.sl as sl
import cv2
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.animation 
import objecttoalg as ob
import nmpcalg as nmpc
import time
import math
import L2_speed_control as sc # closed loop control
import L2_inverse_kinematics as inv # calculates wheel parameters from chassis
import L2_kinematics as kin # gets phi dots
import L2_vector as vec    # calculates chassis speeds from planned velocities
import csv
#def getHeading():
#    quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
#    return euler_from_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

def returned(objs, cams, obj_run, param):
    SIM_TIME = 10.  #total simulation time in (s)
    TIMESTEP = 0.2  #time steps per calculation (Higher Value == larger steps between calculation) orig = 0.1(s) 
    NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP) 
    ROBOT_RADIUS = 0.5   #relative radius of the robot

    objects = objs
    obj_param = param
    err = cams.retrieve_objects(objects, obj_run)
    
    if objects.is_new :
        obj_array = objects.object_list
        print(str(len(obj_array))+" Object(s) detected\n")
        if len(obj_array) > 0 :
            first_object = obj_array[0]
            #print("First object attributes:")
            #print(" Label '"+repr(first_object.label)+"' (conf. "+str(int(first_object.confidence))+"/100)")
            if obj_param.enable_tracking :
                a=0
                #print(" Tracking ID: "+str(int(first_object.id))+" tracking state: "+repr(first_object.tracking_state)+" / "+repr(first_object.action_state))
            positions = first_object.position
            velocity = first_object.velocity
            dimensions = first_object.dimensions
            #posxy = np.array([positions[0],positions[2]])

            #send position and velocity values of objects to alg.
            #obst = ob.createmyobstacle(SIM_TIME, NUMBER_OF_TIMESTEPS, posxy, velocity)
            #print("obst: ", obst)
            #print(" 3D position: [{0},{1},{2}]\n Velocity: [{3},{4},{5}]\n 3D dimentions: [{6},{7},{8}]".format(position[0],position[1],position[2],velocity[0],velocity[1],velocity[2],dimensions[0],dimensions[1],dimensions[2]))
            #print("Act 2D Position: [{0},{1},{2} ]\n".format(positions[0],positions[1],positions[2]))
            return [positions, velocity]

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radian

def cameranmpc(startposition, positionfinal, positionhist, nmpcconfig, zedinfo, objects, sensors_data, obj_runtime_param, obj_param):
    global_target = np.zeros(2)
    global_current = np.zeros(2)
    start = startposition   #******starting point of MY robot **********
    posdesired = positionfinal #********next target point for MY robot ******
    robot_state = positionhist #stored historical position of robots position 
    robothistory = nmpcconfig  #4 = number of predicted horizons My robot and obstacles [0] = prediction horizon [1] = number of time steps
    
    global_disp = 0
    empt1x = [] 
    empt1y = []
    obstx = []
    obsty = []
    timeraw = []
    pdtargetsL = []
    pdtargetsR = []
    while ((abs(robot_state[1] - posdesired[1] )) >=0.25) or ((abs(robot_state[0] - posdesired[0])) >=0.25):
        start = time.time()
        prev_heading = 0
        #prev_pdc = np.zeros(2)
        if zedinfo.grab() == sl.ERROR_CODE.SUCCESS:
            zedinfo.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
            #***** imu 
            #print(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
            #magnetic_field_calibrated = sensors_data.get_magnetometer_data().get_magnetic_field_calibrated()
            #print(" - Magnetometer\n \t Magnetic Field: [ {0} {1} {2} ] [uT]".format(magnetic_field_calibrated[0], magnetic_field_calibrated[1], magnetic_field_calibrated[2]))
            #heading = 90 - math.atan(magnetic_field_calibrated[2]/magnetic_field_calibrated[0])*180/math.pi
            #print("heading:", heading)
            # ***** imu end
            quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
            heading = euler_from_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

            posandvel = returned(objects, zedinfo, obj_runtime_param, obj_param)
            #posxyz = np.zeros(3)
            #velxyz = posxyz
            try:
                posxyz = posandvel[0]
                velxyz = posandvel[1]
            except TypeError:
                #posxyz = np.zeros(3)
                posxyz = [-10,-10,-10]
                velxyz = [0.0001,0.0001,0.0001]
            #posxyz = [1, 0,2]
            #velxyz = [-0.0001,0.0001,0.0001]
            posxy = np.array([posxyz[0],posxyz[2]]) #****x is index 0 y is index 2 ****
            #Accounting for rotation of the camera
            #flippos = np.array([posxy[0], posxy[1]])
            #rotationpos = vec.rotate(flippos, heading[1])
            #modrotation = rotationpos * np.array([0,-1])
            #print("heading:", heading)
            print("Posxy From camera:", posxy)
            #print("rotation", rotationpos)
            #print("mod rot", modrotation)
            velxy = np.array([velxyz[0],velxyz[2]])
            #print("Velxy From Camera:", velxy)
            #plt.scatter(robot_state[0], robot_state[1], color = 'g')
            #***********animation Arrays
            empt1x.append(robot_state[0])
            empt1y.append(robot_state[1])
            obstx.append(posxy[0])
            obsty.append(posxy[1])
            #plt.scatter(posxy[0],posxy[1], color = 'g')  #basic plot of x,y data
            #plt.scatter(robot_state[0], robot_state[1], color = 'b')
            #plt.scatter(modrotation[0],modrotation[1], color = 'r')  #basic plot of x,y data
            obst = ob.createmyobstacle(10, 50, posxy, velxy)
            nmpcalg = nmpc.simulate( obst, robot_state, posdesired, robothistory) #performs the calculation for NMPC alg
            robot_state = nmpcalg #copied value to 
            global_current = global_target
            global_target = robot_state
            print('final;',robot_state)
            print('global current', global_current)
            print('global target',global_target)
            end = time.time()
            timeraw.append((end-start))
            print(end-start)    
            
            #driving
            de_dt = np.zeros(2) # initialize the de_dt

            pdCurrents =  kin.getPdCurrent() # current wheel speeds from kinematics
            cart_target = vec.cart2polar(np.subtract(global_target, global_current), heading[1])
            #cart_target = np.array([0.5,0])
            pdTargets = inv.convert(cart_target)
            
            pdtargetsL.append(pdTargets[0])
            pdtargetsR.append(pdTargets[1])
            #x = 0.0205*(pdCurrents[0]+pdCurrents[1])*(end-start)
            #d_t = 0.10199*(pdCurrents[1]-pdCurrents[0])*(end-start)
            d_t = heading[1] - prev_heading
            #chassis_disp = np.array([x, d_t])
            #cart_disp = np.array([(chassis_disp[0]/chassis_disp[1])*np.cos(chassis_disp[1]), (chassis_disp[0]/chassis_disp[1])*np.sin(chassis_disp[1])])
            cart_disp = (0.0205*(end-start)*(pdCurrents[0] + pdCurrents[1])/d_t)*np.array([np.sin(d_t), -np.cos(d_t)])
            normalized_disp = vec.rotate(cart_disp, prev_heading)
            prev_heading = heading[1]
            global_disp += normalized_disp
            #print("total displacement:", global_disp)
            #print("pdtarget", pdTargets)
            de_dt = 0 # for now, no derivative control

            # CALLS THE CONTROL SYSTEM TO ACTION
            #print("pdcurrent", pdCurrents)

            #sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
            #sc.driveOpenLoop(pdTargets)
                
    #sc.stop()

    return robot_state, empt1x, empt1y, obstx, obsty, timeraw, pdtargetsL, pdtargetsR
    
def _update_plot(i, fig, scat):
    scat.set_xdata(i)
    scat.set_ydata(i)
    return scat,


def main():

    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.sdk_verbose = True

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking=True
    obj_param.image_sync=True
    obj_param.enable_mask_output=True


    camera_infos = zed.get_camera_information()
    if obj_param.enable_tracking :
        positional_tracking_param = sl.PositionalTrackingParameters()
        #positional_tracking_param.set_as_static = True
        positional_tracking_param.set_floor_as_origin = True
        zed.enable_positional_tracking(positional_tracking_param)

    print("Object Detection: Loading Module...")

    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS :
        print (repr(err))
        zed.close()
        exit(1)

    objects = sl.Objects()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40
    
    i=0
    global sensors_data 
    sensors_data = sl.SensorsData()
    #Start the looping process to retrieve camera data & calculate NMPC algorithm


    '''
    start = np.array([0, 0])   #******starting point of MY robot **********
    posdesired = np.array([0, 4]) #********next target point for MY robot ******
    robot_state = start #stored historical position of robots position 
    robothistory = np.empty((4, 200))  #4 = number of predicted horizons My robot and obstacles [0] = prediction horizon [1] = number of time steps
    # predict the obstacles' POSITION in future
    
    try:
        finalpos = cameranmpc(start,posdesired, robot_state, robothistory ,zed, objects, sensors_data, obj_runtime_param, obj_param)
    except:
        sc.stop()
    '''
    
    start = np.array([0, 0])
    posdesired = np.array([0,4])
    robot_state = start
    robothistory = np.empty((4, 200))  #plz changes the 4,200 later on
    finalpos, positionsx1, positionsy1, obx1, oby1, timeraw1, pdL, pdR  =cameranmpc(start, posdesired, robot_state, robothistory ,zed, objects, sensors_data, obj_runtime_param, obj_param) 
    clear_file()
    csv_write(timeraw1, positionsx1, positionsy1, pdL,pdR)
    '''start = finalpos
    posdesired = np.array([4,4])
    robot_state = start
    finalpos, positionsx2, positionsy2, obx2, oby2, timeraw2  =cameranmpc(start, posdesired, robot_state, robothistory ,zed, objects, sensors_data, obj_runtime_param, obj_param)
    start = finalpos
    posdesired = np.array([0,0])
    robot_state = start
    finalpos, positionsx3, positionsy3, obx3, oby3, timeraw3  =cameranmpc(start, posdesired, robot_state, robothistory ,zed, objects, sensors_data, obj_runtime_param, obj_param)

    #appending all list
    totalposx  = positionsx1 + positionsx2 + positionsx3
    totalposy = positionsy1 + positionsy2 + positionsy3
    totalobx = obx1 + obx2 + obx3
    totaloby = oby1 + oby2 + oby3
    totaltime  = timeraw1 + timeraw2 + timeraw3
    #*********animations
    '''
    '''
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=20, metadata=dict(artist='Me'), bitrate=1800)
    fig, ax = plt.subplots()
    #x1, y1 = [0,2,4,6,8,10,12],[0,1,2,3,4,5,6]
    #x2, y2 = [0,1,2,3,4,5,6],[0,1,2,3,4,5,6]
    tmilli = [totaltime * 10 for totaltime in totaltime]
    tmilliround = [round(num) for num in tmilli]
    print(tmilliround)
    frame_t = []
    for i, item in enumerate(tmilliround):
        frame_t.extend([i] * item)
    print(frame_t)
    sc1 = ax.scatter(totalposx,totalposy)
    sc2 = ax.scatter(totalobx,totaloby)
    plt.xlim(-5,5)
    plt.ylim(-1,5)

    def animate(i):

        sc1.set_offsets([totalposx[i], totalposy[i]])
        sc2.set_offsets([totalobx[i], totaloby[i]])
    anim = matplotlib.animation.FuncAnimation(fig, animate, 
                    frames=frame_t, interval=1, repeat=False) 
    #f = r"c://animationtest1.mp4" 
    #writervideo = animation.FFMpegWriter(fps=60) 
    #anim.save(f, writer=writervideo)
    anim.save('ObjectDect1.mp4', writer=writer)
    #plt.show()
    '''
    #plt.xlim(-5,5)
    #plt.ylim(-2,10)
    #plt.draw()

    # Close the camera
    zed.close()
    #sc.stop()
# A function to clear an existing CSV file
def clear_file():
    open('excel_data.csv', 'w').close()


# A function for creating a CSV file from a list of values.
def csv_write(list1, list2, list3, list4, list5):
    list1 = [str(i) for i in list1]
    list2 = [str(i) for i in list2]
    list3 = [str(i) for i in list3]
    list4 = [str(i) for i in list4]
    list5 = [str(i) for i in list5]
    with open('excel_data.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(list1)
        writer.writerow(list2)
        writer.writerow(list3)
        writer.writerow(list4) 
        writer.writerow(list5)
    csvFile.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sc.stop()
