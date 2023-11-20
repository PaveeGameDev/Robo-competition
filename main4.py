#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch
from points.py import points, ordered_points
import time
import math

# Initialize everything
right_motor = Motor(Port.A)
left_motor = Motor(Port.B)
grabber_motor = Motor(Port.C)
# line_sensor = ColorSensor(Port.S1)
# gyro_sensor = GyroSensor(Port.S2)
ev3 = EV3Brick()
# stop_watch = StopWatch()

# constants
# BLACK = 15
# WHITE = 40
# threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 100
# TURN_RATE_DIVIDER = 3
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 150
START_TIME = time.time()
GRAB_SPEED = 500
DROP_OFF_SPEED = 100
TIME_TO_MIDDLE = 80 
TIME_TO_STOP = 83 
DISTANCE_MULTIPLIER = 280


# second initialization
robot = DriveBase(
    left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
)


current_time_from_start = time.time()
gyro_angle = 0
turn_rate_multiplier = 1
wentToMiddle = False
gettingCover = False


# Release variables
TIME_TO_GO_BACK = 5
CUKNOUT_SPEED = 20
RELEASE_WHEELS_WAIT_TIME = 1
RELEASE_WHEELS_SPEED = 100

# Get cover variables
TIME_TO_GET_COVER = 5


def getCover():
    robot.drive_time(-DRIVE_SPEED, TIME_TO_GET_COVER * 1000)
    stop()
    
def dropOff():
    grabber_motor.run_time(CUKNOUT_SPEED, 0.2 * 1000)
    robot.drive_time(-DRIVE_SPEED, TIME_TO_GO_BACK * 1000)
    grabber_motor.run_time(RELEASE_WHEELS_SPEED, RELEASE_WHEELS_WAIT_TIME * 1000)
    getCover()
    
def stop():
    robot.stop()
    grabber_motor.stop()
    ev3.speaker.beep()
    
# go(3.5)
# turn(-90)
# go(7)
# turn(-90)
# go(5)
# turn(-90)
# go(7)
# turn(-90)
# go(2.5)
# turn(-90)
# go(3.5)
dropOff()