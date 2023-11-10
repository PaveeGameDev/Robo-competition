#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch
import time
import math

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
#grabber_motor = Motor(Port.C)
line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
Usonic_sensor = UltrasonicSensor(Port.S3)
ev3 = EV3Brick()
stop_watch = StopWatch()

# second initialization
robot = DriveBase(
    left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
)

while True:
    robot.drive(100,0)

    distance = distance()
    screen.print(distance, end='\n')
    if distance < 100:
        break

print("end of cycle")