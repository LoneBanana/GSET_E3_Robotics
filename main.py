#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import StopWatch, wait
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# Create your objects here.

import math as m
import random as r
import sys as s
import time as t

ev3 = EV3Brick()
# Write your program here.
ev3.speaker.beep()

wait(5000) # wait for 5 seconds before starting stuff 

left_motor = Motor(Port.B)
right_motor = Motor(Port.A)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
line_sensor = ColorSensor(Port.S3)

timer = StopWatch()

wait(3000)

#Test: move forward 10 cm
robot.straight(100) #Move forward 10 cm
ev3.speaker.beep("ISHAN PICK UP YOUR SOCKS!!!")
wait(2000)

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
    right_motor.run(vel_right)

def integral_approx(arr, dt):
    sum = 0
    for i in range(len(arr)):
        sum += 2 * arr[i]
    sum -= arr[0] + arr[len(arr)-1]
    sum *= dt/2
    return sum #returns trapezoidal integral approximartion

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