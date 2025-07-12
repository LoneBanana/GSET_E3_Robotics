#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import StopWatch, wait
from pybricks.nxtdevices import LightSensor
from pybricks.tools import DataLog
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# Create your objects here.


import math as m
import random as r
import sys as s
import time as t


ev3 = EV3Brick()


err_arr = []
correction = 0
in_maze=False

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)


robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)




light_sensor = LightSensor(Port.S3)
color_sensor = ColorSensor(Port.S2)
distance_sensor = UltrasonicSensor(Port.S1)

timer = StopWatch()
elapsed_time = timer.time()
detected_color = color_sensor.color()
distance = distance_sensor.distance()

left_motor.stop(Stop.BRAKE)
right_motor.stop(Stop.BRAKE)


def drive(speed, distance):
    wheel_diameter_cm = 5.55
    circumference = m.pi * wheel_diameter_cm
    for_degrees = (distance / circumference) * 360  # Calculate the degrees to rotate the wheels
    left_motor.run_angle(speed, for_degrees, wait=False)
    right_motor.run_angle(speed, for_degrees, wait=True)  # Ensure both motors complete the movement
def integral_approx(arr, dt):
    sum = 0
    for i in range(len(arr)):
        sum += 2 * arr[i]
    sum -= arr[0] + arr[len(arr)-1]
    sum *= dt/2
    return sum #returns trapezoidal integral approximartion
def follow_line(speed, target_intensity, k_p, k_i, dt, socks, brownies):
    """
    Follows a line using a proportional control algorithm.
   
    :param speed: Base speed of the robot.
    :param target_reflection: Target reflection value (e.g., 50 for the edge of the line).
    :param k_p: Proportional gain for the control.
    """
    # Get the current reflection value from the line sensor
    light_intensity = light_sensor.reflection()
   
    # Calculate the error (difference between target and current reflection)
    error = target_intensity - light_intensity
   
    # Calculate the correction using proportional control
    correction = k_p * error
    err_arr.append(error)
   
    avg = sum(err_arr)/len(err_arr)
   
    if len(err_arr) > socks:
     err_arr.pop(0)
    correction += k_i * integral_approx(err_arr, dt) # 100 ms
   
    # Adjust motor speeds based on the correction
    left_motor_speed = speed + correction
    right_motor_speed = speed - correction
    """
    if (m.abs(avg) <= brownies and len(err_arr) > 0.8 * socks):
        ev3.speaker.beep()
        left_motor.run(left_motor_speed * 1.2)
        right_motor.run(right_motor_speed * 1.2)
    else:
    """
    # Run the motors with the calculated speeds
    left_motor.run(left_motor_speed)
    right_motor.run(right_motor_speed)
def navigate_maze():
    if distance < 30:
        left_motor.run(300)
        right_motor.run(300)
    else:
        left_motor.stop()
        right_motor.stop()


# Initialize the DataLog
log = DataLog("time", "voltage", name="light_sensor_log", timestamp=True)
ev3.speaker.beep()


while True:
    if detected_color == Color.RED:
        in_maze=True
    if not in_maze:
        if elapsed_time < 5000:
            follow_line(230,46,0.5, 0.5,0.1,60,50)
        if elapsed_time > 20000:
            follow_line(230,46,0.5, 0.5,0.1,60,50)
        else:
            follow_line(200,46,1.0, 0.5,0.1,60,50)
    elif in_maze:
        navigate_maze()
    
    #In honor of our lord and savior Supreme Leader Czar Ishaan Kunwar"
    #: socks = length of err_arr
    #: brownies = mximum average error of array we are willing to consider before speed
   
    """ Derivative Code:
   




    """
   
'''
def p_drive(theta_d, k_p, epsilon=1.0): #epsilon = how close to be before error is considered negligible
    #declare max and min values of power
    MAX_POWER = 100
    MIN_POWER = -100


    theta_a = robot.angle()
    err = k_p(theta_d - theta_a) #Make sure error = desires - actual
   
    if (abs(err) <= epsilon):
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        return
   
    wheel_power = err #error term is the wheel power
    wheel_power = max(MIN_POWER, min(wheel_power, MAX_POWER))
   
    robot.drive(wheel_power, 0) #If we notice we happen to have turn, then we can add two separate error terms


def degToDistance(deg):
    return (deg * m.pi/180) * m.pi * 5.55 #5.55 is necessary because conversion from mm --> cm


def p_follow_line(k_p, wheel_power, target_reflection):
   
    reflect = line_sensor.reflection()
    err = k_p*(target_reflection - reflect) #we want target_reflection = 50 because edge of line is 50
    vel_left = wheel_power + err
    vel_right = wheel_power - err
    left_motor.run(vel_left)
    right_motor.run(vel_rigrake()ht)


def p_i_follow_line(k_p, k_i, wheel_power, target_reflection, dt):
    err_arr=[]
    reflect = line_sensor.reflection()
    err = k_p * (target_reflection - reflect) + k_i * integral_approx(err_arr, dt) #dt in milliseconds
    err_arr.append(err)
    val_left = wheel_power + err
    val_right = wheel_power - err
    left_motor.run(val_left)
    right_motor.run(val_right)


#-----------




default_power = 10 #don't know if that's sufficient
target_reflection = 50 #we want to follow the edge of the line, which is at 50
k_p = 0.1 #change as we go
robot.drive(default_power, 0)


last = 0
while True:
    now = timer.time()
    if (now - last >= 100):
        p_drive(100, 0.1)
        last = now


'''

