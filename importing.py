#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch
from points import points, ordered_points

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
# grabber_motor = Motor(Port.C)
#line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
USsensor = UltrasonicSensor(Port.S3)
ev3 = EV3Brick()
stop_watch = StopWatch()

#contants
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 150

# second initialization
robot = DriveBase(
    left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
)