#!/usr/bin/env pybricks-micropython
from importing import robot,  ev3, stop_watch,gyro_sensor #grabber_motor, line_sensor , USsensor,
#from DPS import DPS_class
import time
import math

def a_turn():
    robot.drive(-0.5*math.pi*140,90)
    while True:
        if gyro_sensor.angle() <= -90:
            robot.stop()
            ev3.speaker.beep()
            print(gyro_sensor.angle())
            break
        
def b_turn():
    robot.drive(-1.5*math.pi*140,-90)
    while True:
        if gyro_sensor.angle() >= 90:
            robot.stop()
            ev3.speaker.beep()
            print(gyro_sensor.angle())
            break
""" 
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

def go(distance, currentGyro):
    robot.reset()
    gyro_sensor.reset_angle(currentGyro)
    while robot.distance() < distance * DISTANCE_MULTIPLIER:
        robot.drive(100, -gyro_sensor.angle())
        print("distance", robot.distance())
        print("gyro", gyro_sensor.angle())
        wait(50)
    
    print("going straight for", distance * DISTANCE_MULTIPLIER)


def turn(angle):
    global gyroAngle
    global gyroOffset
    print("turning", angle)
    robot.turn(angle)
    currentgyroError = gyrogyro_sensor.angle() - angle - gyroAngle
    print("gyro error", currentgyroError)
    print("gyro offset", gyroOffset)
    gyroAngle = gyrogyro_sensor.angle()
    gyroOffset += currentgyroError

grabber_motor.dc(-100)
go(1, 10)
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
dropOff() """