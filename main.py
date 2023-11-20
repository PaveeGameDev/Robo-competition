#!/usr/bin/env pybricks-micropython
from importing import robot, gyro_sensor, USsensor, ev3, stop_watch, #grabber_motor, line_sensor
from DPS import DPS_class
import time
import math

WHEEL_DIAMETER = 40
AXLE_TRACK = 200
START_TIME = time.time()
GRAB_SPEED = 10000
DROP_OFF_SPEED = 100
TIME_TO_MIDDLE = 80 
TIME_TO_STOP = 83 
DISTANCE_MULTIPLIER = 280
START_POINT = [0,0,0] ##TODO get the position
SIDE_OF_US = -1
US_OFFSET = [-85,40] ##TODO get the numbers
#-#-# Release variables
TIME_TO_GO_BACK = 5
CUKNOUT_SPEED = 20
RELEASE_WHEELS_WAIT_TIME = 0.5
RELEASE_WHEELS_SPEED = 100

# Get cover variables
TIME_TO_GET_COVER = 5
current_time_from_start = time.time()
gyro_angle = 0
turn_rate_multiplier = 1
wentToMiddle = False
gettingCover = False

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

DPS = DPS_class(x = START_POINT[0], y = START_POINT[1], angle = START_POINT[0])
# grabber_motor.run(-GRAB_SPEED)
# grabber_motor.dc(-100)
DPS.go_by_wall([5.5 * 280, 0], 140, SIDE_OF_US, US_OFFSET)
""" 
DPS.turn(90)
DPS.go_by_wall([0, 5 * 280], 280, SIDE_OF_US, US_OFFSET)
DPS.turn(90)
DPS.go_by_wall([7 * 280, 0], 140, SIDE_OF_US, US_OFFSET)
DPS.turn(90)
DPS.go_by_wall([0, 5 * 280], 280, SIDE_OF_US, US_OFFSET)
DPS.turn(90) """