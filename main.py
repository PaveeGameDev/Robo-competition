#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch
import time
import math

# Initialize everything
right_motor = Motor(Port.A)
left_motor = Motor(Port.B)
grabber_motor = Motor(Port.C)
touchSensor = TouchSensor(Port.S4)
# line_sensor = ColorSensor(Port.S1)
# gyro_sensor = GyroSensor(Port.S2)
ev3 = EV3Brick()
# stop_watch = StopWatch()

# constants
# BLACK = 15
# WHITE = 40
# threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 200
# TURN_RATE_DIVIDER = 3
WHEEL_DIAMETER = 30
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
RELEASE_WHEELS_WAIT_TIME = 0.5
RELEASE_WHEELS_SPEED = 100

# Get cover variables
TIME_TO_GET_COVER = 5


def getCover():
    robot.drive(DRIVE_SPEED / 6, 0)
    wait(500)
    stop()
    
def dropOff():
    grabber_motor.run_time(1000, 0.4 * 1000, then=Stop.HOLD, wait=True)
    grabber_motor.run_time(-1000, 1 * 1000, then=Stop.HOLD, wait=True)
    while touchSensor.pressed():
        robot.drive(DRIVE_SPEED / 6, 0)
        
    robot.drive(DRIVE_SPEED / 6, 0)
    wait(1000)
    grabber_motor.run_time(1000, RELEASE_WHEELS_WAIT_TIME * 1000, then=Stop.HOLD, wait=False)
    robot.drive(DRIVE_SPEED / 6, 0)
    wait(500)
    # getCover()
    
def stop():
    robot.stop()
    grabber_motor.stop()
    ev3.speaker.beep()
    
    
def go(distance):
    robot.straight(distance * DISTANCE_MULTIPLIER, then=Stop.HOLD, wait=True)
    print("going straight for", distance * DISTANCE_MULTIPLIER)


def turn(angle):
    robot.turn(angle)
    
# grabber_motor.run(-GRAB_SPEED)
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
# turn(-90)
# wait(10)
dropOff()