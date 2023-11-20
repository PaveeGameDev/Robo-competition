#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# grabber_motor = Motor(Port.C)
#line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S1)
#USsensor = UltrasonicSensor(Port.S3)
ev3 = EV3Brick()
stop_watch = StopWatch()

#contants
WHEEL_DIAMETER = 42
AXLE_TRACK = 210

# second initialization
robot = DriveBase(
    left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
)