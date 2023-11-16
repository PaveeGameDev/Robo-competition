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
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
# grabber_motor = Motor(Port.C)
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


def getCover():
    global gettingCover
    gettingCover = True
    print("getting cover")
    while True:
        left_motor.run(-DRIVE_SPEED)
        right_motor.run(-DRIVE_SPEED)


def dropOff():
    print("dropping off")
    grabber_motor.run_time(DROP_OFF_SPEED, 5 * 1000, then=Stop.HOLD, wait=True)
    getCover()


# def needToGoMiddle():
#     global wentToMiddle
#     wentToMiddle = True
#     dropOff()


def go(distance):
    robot.straight(distance * DISTANCE_MULTIPLIER)


def turn(angle):
    robot.turn(angle)


go(1)
turn(-90)
go(3.5)
turn(-90)
go(7)
turn(-90)
go(5)
turn(-90)
go(7)
turn(-90)
go(2.5)
turn(-90)
go(3.5)
dropOff()
getCover()

while True:
    if time.time() > current_time_from_start + TIME_TO_STOP:
        ev3.speaker.beep()
        break
