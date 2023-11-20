#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch
import time
import math

# Initialize everything
right_motor = Motor(Port.B)
left_motor = Motor(Port.C)
grabber_motor = Motor(Port.D)
# touchSensor = TouchSensor(Port.S4)
# line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S1)
ev3 = EV3Brick()
# stop_watch = StopWatch()

# constants
# BLACK = 15
# WHITE = 40
# threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 200
# TURN_RATE_DIVIDER = 3
WHEEL_DIAMETER = 40
AXLE_TRACK = 200
START_TIME = time.time()
GRAB_SPEED = 10000
DROP_OFF_SPEED = 100
TIME_TO_MIDDLE = 80 
TIME_TO_STOP = 83 
DISTANCE_MULTIPLIER = 280


# second initialization
# robot = DriveBase(
#     left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
# )


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

# Going variables
gyroAngle = 0
gyroOffset = 0

def getCover():
    robot.drive(DRIVE_SPEED / 3, 0)
    wait(500)
    stop()
    
def dropOff():
    grabber_motor.run_time(1000, 0.4 * 1000, then=Stop.HOLD, wait=True)
    grabber_motor.run_time(-1000, 1 * 1000, then=Stop.HOLD, wait=True)
    while touchSensor.pressed():
        robot.drive(DRIVE_SPEED / 3, 0)
        
    robot.drive(DRIVE_SPEED / 3, 0)
    wait(2000)
    grabber_motor.run_time(1000, RELEASE_WHEELS_WAIT_TIME * 1000, then=Stop.HOLD, wait=False)
    robot.drive(DRIVE_SPEED / 3, 0)
    wait(500)
    getCover()
    
def stop():
    robot.stop()
    grabber_motor.stop()
    ev3.speaker.beep()
    
    
def go(distance):
    robot.straight(-distance * DISTANCE_MULTIPLIER)
    
    print("going straight for", distance * DISTANCE_MULTIPLIER)


def turn(angle):
    global gyroAngle
    global gyroOffset
    print("turning", angle)
    robot.turn(angle)
    currentgyroError = gyro_sensor.angle() - (angle + gyroAngle)
    print("gyro error", currentgyroError)
    print("gyro offset", gyroOffset)
    gyroAngle = gyro_sensor.angle()
    gyroOffset += currentgyroError
    
# grabber_motor.run(-GRAB_SPEED)
grabber_motor.dc(-100)
left_motor.dc(-100)
right_motor.dc(-100)
wait(20000)
# go(10)
# go(3.5)
# turn(90)
# go(7)
# turn(90)
# go(5)
# turn(90)
# go(7)
# turn(90)
# go(2.5)
# turn(90)
# go(3.5)
# turn(90)
# wait(10)
dropOff()