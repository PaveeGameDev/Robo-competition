#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
import time

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
ev3 = EV3Brick()


# constants
BLACK = 5
WHITE = 30
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 10
TURN_RATE_DIVIDER = 3
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 162
START_TIME = time.time()


# second initialization
robot = DriveBase(
    left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
)

robot.turn(360)
