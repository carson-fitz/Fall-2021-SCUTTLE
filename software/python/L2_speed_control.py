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
    u = np.round((u_proportional + u_integral + u_derivative), 2)   # must round to ensure driver handling
    u = scaleMotorEffort(u)                                         # perform scaling - described above
    u[0] = sorted([-1, u[0], 1])[1]                                 # place bounds on the motor commands
    u[1] = sorted([-1, u[1], 1])[1]                                 # within [-1, 1]

    # SEND SIGNAL TO MOTORS
    m.sendLeft(round(u[0], 2))                                        # must round to ensure driver handling!
    m.sendRight(round(u[1], 2))                                        # must round to ensure driver handling!
    return

if __name__ == "__main__":
    # IMPORT EXTERNAL ITEMS
    from timeit import default_timer as timer
    from time import sleep

    # IMPORT INTERNAL ITEMS
    import L2_speed_control as sc # closed loop control. Import speed_control for open-loop
    import L2_inverse_kinematics as inv #calculates wheel parameters from chassis
    import L2_vector as vec    # calculates chassis parameters from wheels

    # CREATE A FUNCTION FOR DRIVING
    def loop_drive():
        # INITIALIZE VARIABLES FOR CONTROL SYSTEM
        t0 = 0  # time sample
        t1 = 1  # time sample
        e00 = 0 # error sample
        e0 = 0  # error sample
        e1 = 0  # error sample
        dt = 0  # delta in time
        de_dt = np.zeros(2) # initialize the de_dt
        global_vel = 0      # this gets set by the path planner
        heading = 0         # heading comes from compass
        
        while(1):
            # THIS CODE IS FOR OPEN AND CLOSED LOOP control
            pdTargets = inv.convert(vec.cart2polar(global_vel, heading)) # Input requested PhiDots (radians/s)
            pdCurrents = inv.convert(vec.cart2polar(global_vel, heading)) # parameters will come from path planner
            '''
            # THIS BLOCK UPDATES VARIABLES FOR THE DERIVATIVE CONTROL
            t0 = t1  # assign t0
            t1 = timer() # generate current time
            dt = t1 - t0 # calculate dt
            e00 = e0 # assign previous previous error
            e0 = e1  # assign previous error
            e1 = pdCurrents - pdTargets # calculate the latest error
            de_dt = (e1 - e0) / dt # calculate derivative of error
            ''' 
            de_dt = 0 # for now, no derivative control

            # CALLS THE CONTROL SYSTEM TO ACTION
            sc.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
            sleep(0.05) # this time controls the frequency of the controller
                
    loop_drive() # call the function    
