# Taken from SCUTTLE w/ BB-Blue code
# speed_control.py takes target speeds and generates duty cycles
# to send to motors, and has a function to execute PID control.

# Import external libraries
import numpy as np                                  # for handling arrays

# Import local files
import L1_motor_Jetson as m                               # for controlling motors

# Initialize variables
u_integral = 0
#DRS = 0.8                                           # direct rescaling - for open-loop motor duty
kp = 0.04                                           # proportional term
ki = 0.04                                           # integral term
kd = 0.0                                            # derivative term
pidGains = np.array([kp, ki, kd])                   # form an array to collect pid gains.

def stop():
    m.stop()

# a function for converting target rotational speeds to PWMs without feedback
def openLoop(pdl, pdr):
    DRS = 0.99                                      # create a variable for direct-re-scaling
    duties = np.array([pdl, pdr])                   # put the values into an array
    duties = duties * 1/9.75 * DRS                  # rescaling. 1=max PWM, 9.75 = max rad/s achievable
    duties[0] = sorted([-0.99, duties[0], 0.99])[1]       # place bounds on duty cycle
    duties[1] = sorted([-0.99, duties[1], 0.99])[1]       # place bounds on duty cycle
    return duties

def driveOpenLoop(pdTargets):                       # Pass Phi dot targets to this function
    duties = openLoop(pdTargets[0], pdTargets[1])   # produce duty cycles from the phi dots
    m.sendLeft(duties[0])                           # send command to motors
    m.sendRight(duties[1])                          # send command to motors

def scalingFunction(x):                             # a fcn to compress the PWM region where motors don't turn
    if -0.222 < x and x < 0.222:
        y = (x * 3)
    elif x > 0.222:
        y = ((x * 0.778) + 0.222)
    else:
        y = ((x * 0.778) - 0.222)
    return y

def scaleMotorEffort(u):                            # send the control effort signals to the scaling function
    u_out = np.zeros(2)
    u_out[0] = scalingFunction(u[0])
    u_out[1] = scalingFunction(u[1])
    return(u_out)

def driveClosedLoop(pdt, pdc, de_dt):               # this function runs motors for closed loop PID control
    global u_integral
    e = (pdt - pdc)                                 # compute error

    kp = pidGains[0]                                # gains are input as constants, above
    ki = pidGains[1]
    kd = pidGains[2]

    # GENERATE COMPONENTS OF THE CONTROL EFFORT, u
    u_proportional = (e * kp)                                       # proportional term
    try:
        u_integral += (e * ki)                                      # integral term
    except:
        u_integral = (e * ki)                                       # for first iteration, u_integral does not exist

    u_derivative = (de_dt * kd)                                     # derivative term takes de_dt as an argument

    # CONDITION THE SIGNAL BEFORE SENDING TO MOTORS
    u = (u_proportional + u_integral + u_derivative)  
    u = scaleMotorEffort(u)                                         # perform scaling - described above
    u[0] = np.round(sorted([-1, u[0], 1])[1], 2)                                # place bounds on the motor commands
    u[1] = np.round(sorted([-1, u[1], 1])[1], 2)                                 # within [-1, 1]

    print("motors", u)

    # SEND SIGNAL TO MOTORS
    m.sendLeft(u[0])                                        
    m.sendRight(u[1])                                        # no longer rounded to 2, already got rounded anyway
    

if __name__ == "__main__":
    # IMPORT EXTERNAL ITEMS
    from timeit import default_timer as timer
    from time import sleep

    # IMPORT INTERNAL ITEMS
    import objectdetection as ob # zed object detection, posts IMU data to global variables
    import L2_speed_control as sc # closed loop control
    import L2_inverse_kinematics as inv # calculates wheel parameters from chassis
    import L2_kinematics as kin # gets phi dots
    import L2_vector as vec    # calculates chassis speeds from planned velocities

    # CREATE A FUNCTION FOR DRIVING
    ob.main()
    def loop_drive():
        # INITIALIZE VARIABLES FOR CONTROL SYSTEM
        de_dt = np.zeros(2) # initialize the de_dt
        # THIS CODE IS FOR OPEN AND CLOSED LOOP control
        # parameters will come from path planner as globals
        # difference between target and current is global displacement, for now current can just be the previous waypoint but
        # later it needs to be set by IMU's pose estimation or some other localization method
        heading, current, target = ob.getGlobals()
        cart_target = vec.cart2polar(np.subtract(target, current), heading)
        pdTargets = inv.convert(cart_target)

        pdCurrents =  kin.getPdCurrent() # current wheel speeds from kinematics

        de_dt = 0 # for now, no derivative control

        # CALLS THE CONTROL SYSTEM TO ACTION
        sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
        sleep(0.05) # this time controls the frequency of the controller
                
    #loop_drive() # call the function    
    
