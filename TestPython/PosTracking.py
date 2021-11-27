import pyzed.sl as sl 
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation


fig = plt.figure()
def main():

    #create camera
    zed = sl.Camera()

    #create InitParameters object and config
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.coordinate_units = sl.UNIT.METER

    #open Camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
            exit(1)
    
    #inable pos tracking w/ params set to default
    py_transform = sl.Transform() #createing trasform object.
    tracking_parameters = sl.PositionalTrackingParameters(_init_pos = py_transform)
    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
    
    #Track for 1000 Frames
    i = 0 
    zed_pose = sl.Pose()

    zed_sensors = sl.SensorsData()
    runtime_parameters = sl.RuntimeParameters()

    while i < 200:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            #get pose of the left eye w/ reference to world frame
            zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            zed.get_sensors_data(zed_sensors, sl.TIME_REFERENCE.IMAGE)
            zed_imu = zed_sensors.get_imu_data()

            #translation info & time
            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0],3)
            ty = round(zed_pose.get_translation(py_translation).get()[1],3)
            tz = round(zed_pose.get_translation(py_translation).get()[2],3)
            #print("Trans: Tx: {0}, Ty: {1}, Tz, {2}, TIMESTAMP: {3}\n".format(tx,ty,tz,zed_pose.timestamp.get_milliseconds()))
            i = i +1
            #plt.scatter(tx,tz)  #basic plot of x,y data
            #line, = plt.plot(tx,ty) #basic plot of x,y data
            

        
    #plt.draw()
    #plt.show()       
    zed.close() #turn off camera



if __name__ == "__main__":
    main()
    #animation = FuncAnimation(fig, animate, interval =20)

