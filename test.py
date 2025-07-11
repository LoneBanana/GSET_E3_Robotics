from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import StopWatch, wait

ev3 = EV3Brick()

ev3.speaker.beep()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

left_motor.stop()

def drive(power):
    left_motor(power)
    right_motor(power)

wait(1000)
drive(100)
wait(1000)

