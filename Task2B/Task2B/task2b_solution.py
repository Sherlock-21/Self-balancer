#python
import sys
import numpy as np
import math
###### GLOBAL VARIABLES HERE ######
base = None
motor = None
arm = None
pendulum = None
U = None
yaw_setpoint = 0




def sysCall_init():
    # do some initialization here
    global K
    ####### ADD YOUR CODE HERE ######
    global pendulum_joint_handle, base_handle, motor_handle , ref


    pendulum_joint_handle = sim.getObjectHandle('bike_respondable')
    ref = sim.getObjectHandle("reference_frame")
    motor_handle = sim.getObjectHandle('front_motor') 
    pass

def sysCall_actuation():
       
    global yaw_setpoint
    yaw_setpoint = sim.getFloatSignal("yaw_setpoint")
    #print(math.degrees(yaw_setpoint))
    desired_alpha_dot = 0
    #desired_alpha = math.degrees(yaw_setpoint)
    desired_theta_dot = 0
    desired_theta = 90
    
    theta = sim.getJointPosition(motor_handle)
    theta_dot = sim.getJointVelocity(motor_handle)
    
    
    m=sim.getObjectMatrix(pendulum_joint_handle,ref)
    run=math.sqrt((m[0]*m[0])+(m[4]*m[4]))
    rise=m[8]
    
    theta=math.atan2(run,rise)
    thetad=math.degrees(theta)
    
    alpha = math.atan2(m[4], m[0])
    alphad = math.degrees(alpha)
    
    
    
    linear, angular = sim.getObjectVelocity(pendulum_joint_handle)
    alpha_dot = angular[2]
    theta_dot = angular[0]
    
    x1 = desired_alpha_dot - alpha_dot
    x2 = 30 - alphad
    x3 = desired_theta_dot - theta_dot
    x4 = -desired_theta + thetad
    print(x1)
    y=sim.getObjectPosition(pendulum_joint_handle,ref)
    x5= -0.100 + y[1]
    
    
    
    Kp = 5
    Ki = 0
    Kd =0
    error_integral = 0.0  
    previous_error = 0.0  

    
    error = x4 +x2
    error_integral += error
    error_derivative = error - previous_error

    
    u1 = +Kp * error + Ki * error_integral + Kd * error_derivative
    
    Kpa = 0.001
    Kia = 0
    Kda = 0.1
    error_integrala = 0.0  
    previous_errora = 0.0  

    
    errora = x2
    error_integrala += errora
    error_derivativea = errora - previous_errora
    u2 = +Kpa * errora + Kia * error_integrala + Kda * error_derivativea
    
    
    
    u = u1

    
    print(x2)
    sim.setJointTargetVelocity(motor_handle, (1*u))
    previous_error = error
    previous_errora = errora
    pass

def sysCall_sensing():
    # put your sensing code here
    global yaw_setpoint
    yaw_setpoint = sim.getFloatSignal("yaw_setpoint")
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
