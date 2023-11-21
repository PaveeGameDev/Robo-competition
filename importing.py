#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
grabber_motor = Motor(Port.D)
gyro_sensor = GyroSensor(Port.S1)
touchSensor = TouchSensor(Port.S2)
ev3 = EV3Brick()
stop_watch = StopWatch()

#contants
WHEEL_DIAMETER = 42
AXLE_TRACK = 210

# second initialization
robot = DriveBase(
    left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
)