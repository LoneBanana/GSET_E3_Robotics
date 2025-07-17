#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()
distance_sensor = UltrasonicSensor(Port.S1)
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
small_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)


ev3.light.on(Color.YELLOW)
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
ev3.light.on(Color.GREEN)
ev3.speaker.beep()
wait(5000)


#PUT YOUR CODE HERE  Delete mine
while True:
    if distance_sensor.distance() > 100:
        left_motor.run(300)
        right_motor.run(-300)
    else:
        ev3.speaker.beep()
        left_motor.run(500)
        right_motor.run(500)
