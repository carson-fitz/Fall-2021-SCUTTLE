from numpy.lib.function_base import piecewise
import pyzed.sl as sl
import cv2
import numpy as np
import matplotlib.pyplot as plt 
import objecttoalg as ob
import nmpcalg as nmpc
import time
import math

global_heading = 0
global_current = np.array([0,0])
global_target = np.array([0,0])

def getGlobals():
    return global_heading[0], global_current, global_target

def returned(objs, cams, obj_run, param):
    SIM_TIME = 10.  #total simulation time in (s)
    TIMESTEP = 0.5  #time steps per calculation (Higher Value == larger steps between calculation) orig = 0.1(s) 
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
    global global_heading, global_target, global_current
    start = startposition   #******starting point of MY robot **********
    posdesired = positionfinal #********next target point for MY robot ******
    robot_state = positionhist #stored historical position of robots position 
    robothistory = nmpcconfig  #4 = number of predicted horizons My robot and obstacles [0] = prediction horizon [1] = number of time steps
    while ((abs(robot_state[1] - posdesired[1] )) >=0.25) or ((abs(robot_state[0] - posdesired[0])) >=0.25):
        start = time.time()
        if zedinfo.grab() == sl.ERROR_CODE.SUCCESS:
            zedinfo.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
            #***** imu 
            quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
            
            global_heading = euler_from_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            #print(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
            #magnetic_field_calibrated = sensors_data.get_magnetometer_data().get_magnetic_field_calibrated()
            #print(" - Magnetometer\n \t Magnetic Field: [ {0} {1} {2} ] [uT]".format(magnetic_field_calibrated[0], magnetic_field_calibrated[1], magnetic_field_calibrated[2]))
            #heading = 90 - math.atan(magnetic_field_calibrated[2]/magnetic_field_calibrated[0])*180/math.pi
            #print("heading:", heading)
            # ***** imu end
            
            posandvel = returned(objects, zedinfo, obj_runtime_param, obj_param)
            posxyz = posandvel[0]
            velxyz = posandvel[1]
            posxy = np.array([posxyz[0],posxyz[2]]) #****x is index 0 y is index 2 ****
            #print("Posxy From camera:", posxy)
            velxy = np.array([velxyz[0],velxyz[2]])
            #print("Velxy From Camera:", velxy)
            #plt.scatter(robot_state[0], robot_state[1], color = 'b')
            #plt.scatter(posxy[0],posxy[1], color = 'r')  #basic plot of x,y data
            obst = ob.createmyobstacle(10, 200, posxy, velxy)
            nmpcalg = nmpc.simulate( obst, robot_state, posdesired, robothistory) #performs the calculation for NMPC alg
            robot_state = nmpcalg #copied value to 
            global_current = global_target
            global_target = robot_state
            print('final;',robot_state)
            end = time.time()
            print(end-start)
    return robot_state
    


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
    sensors_data = sl.SensorsData()
    #Start the looping process to retrieve camera data & calculate NMPC algorithm



    start = np.array([0, 0])   #******starting point of MY robot **********
    posdesired = np.array([0, 2.5]) #********next target point for MY robot ******
    robot_state = start #stored historical position of robots position 
    robothistory = np.empty((4, 200))  #4 = number of predicted horizons My robot and obstacles [0] = prediction horizon [1] = number of time steps
    # predict the obstacles' POSITION in future
    finalpos = cameranmpc(start,posdesired, robot_state, robothistory ,zed, objects, sensors_data, obj_runtime_param, obj_param)
    start = finalpos

    posdesired = np.array([0,5])
    robot_state = start
    finalpos =cameranmpc(start, posdesired, robot_state, robothistory ,zed, objects, sensors_data, obj_runtime_param, obj_param)
   # start = finalpos
   # posdesired = np.array([2,6])
   # robot_state = start
   # finalpos =cameranmpc(start, posdesired, robot_state, robothistory ,zed, objects, sensors_data, obj_runtime_param, obj_param)
   
    #plt.xlim(-5,5)
    #plt.ylim(-1,11)
    #plt.draw()
    #plt.show()

    # Close the camera
    zed.close()
    

if __name__ == "__main__":
    main()
