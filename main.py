#!/usr/bin/env pybricks-micropython
from importing import robot,  ev3, stop_watch,gyro_sensor, grabber_motor, touchSensor, wait, Stop
#from DPS import DPS_class
import time
import math

def a_turn():
    robot.drive(-0.5*math.pi*140,90)
    beggining_angle = gyro_sensor.angle()
    while True:
        if gyro_sensor.angle() <= -90 - beggining_angle:
            robot.stop()
            return gyro_sensor.angle() - (-90 - beggining_angle)
        
def b_turn():
    robot.drive(-1.5*math.pi*140,90)
    beggining_angle = gyro_sensor.angle()
    while True:
        if gyro_sensor.angle()  <= -90 - beggining_angle:
            robot.stop()
            return gyro_sensor.angle() - (-90 - beggining_angle)

WHEEL_DIAMETER = 40
AXLE_TRACK = 200
START_TIME = time.time()
GRAB_SPEED = 10000
DROP_OFF_SPEED = 100
TIME_TO_MIDDLE = 80 
TIME_TO_STOP = 83 
DISTANCE_MULTIPLIER = 280
SIDE_OF_US = -1
US_OFFSET = [-85,40] 
DRIVE_SPEED = 100

#-#-# Release variables
TIME_TO_GO_BACK = 5
CUKNOUT_SPEED = 20
RELEASE_WHEELS_WAIT_TIME = 4
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

def turnA():
    robot.drive(-0.5*math.pi*140,90)
    beggining_angle = gyro_sensor.angle()
    while True:
        if gyro_sensor.angle() <= -90 - beggining_angle:
            robot.stop()
            return gyro_sensor.angle() - (-90 - beggining_angle)

    return 0
        
def turnB():
    robot.drive(-1.5*math.pi*140,90)
    beggining_angle = gyro_sensor.angle()
    while True:
        if gyro_sensor.angle()  <= -90 - beggining_angle:
            robot.stop()
            return gyro_sensor.angle() - (-90 - beggining_angle)

def getCover():
    robot.drive(DRIVE_SPEED / 3, 0)
    wait(2000)
    stop()
    
def dropOff():
    print("drop off")
    grabber_motor.dc(100)
    wait(300)
    grabber_motor.dc(-100)
    wait(300)
    grabber_motor.dc(100)
    wait(300)
    grabber_motor.dc(-100)
    wait(2000)
    grabber_motor.dc(0)
    grabber_motor.run_time(-10000, 0.5 * 1000, then=Stop.HOLD, wait=False)
    while not touchSensor.pressed():
        robot.drive(DRIVE_SPEED / 3, 0)
    robot.drive(DRIVE_SPEED / 3, 0)
    wait(2500)
    robot.drive(-DRIVE_SPEED / 3, 0)
    wait(1000)
    grabber_motor.dc(100)
    wait(2200)
    # grabber_motor.run_time(10000, RELEASE_WHEELS_WAIT_TIME * 1000, then=Stop.HOLD, wait=True)
    print("drop off done")
    getCover()
    
def stop():
    robot.stop()
    grabber_motor.stop()
    ev3.speaker.beep()

def go(distance, currentGyro):
    print("going straight for", distance * DISTANCE_MULTIPLIER)
    robot.reset()
    gyro_sensor.reset_angle(currentGyro)
    while abs(robot.distance()) < abs(distance * DISTANCE_MULTIPLIER):
        robot.drive(-100 * abs(distance)/distance, gyro_sensor.angle())
        print("distance", robot.distance())
        print("gyro", gyro_sensor.angle())
        wait(50)
    
    
# def alternativeGo(distance, currentGyro):
#     print("going straight for", distance * DISTANCE_MULTIPLIER)
#     robot.reset()
#     gyro_sensor.reset_angle(currentGyro)
#     robot.stop()
#     while abs(robot.distance()) < abs(distance * DISTANCE_MULTIPLIER):
#         left_motor.run(DRIVE_SPEED * abs(distance)/distance)
#         right_motor.run(DRIVE_SPEED * abs(distance)/distance)
#         print("distance", robot.distance())
#         print("gyro", gyro_sensor.angle())
#         wait(50)
    
def folow_wall(target):
    distance = USsensor.distance()
    
    if distance - 20 > target:
        robot.turn(10)
        robot.drive(-100, 0)
    elif distance + 20 < target:
        robot.turn(-10)
        robot.drive(-100,0)
    else:
        robot.drive(-100,0)

def turn(angle):
    global gyroAngle
    global gyroOffset
    print("turning", angle)
    robot.turn(angle)
    currentgyroError = gyro_sensor.angle() - angle - gyroAngle
    print("gyro error", currentgyroError % 90 - 90)
    print("gyro offset", gyroOffset)
    gyroAngle = gyro_sensor.angle()
    gyroOffset += currentgyroError
    return currentgyroError % 90 - 90

grabber_motor.dc(-100)
wait(1000)
go(3.5, 0)
firstOffset = turn(45)
go(0.7,0)
secondError = turn(abs(45 + firstOffset))
go(4.5, 0)
firstOffset = turn(22)
go(2.061,0)
secondError = turn(abs(78 + firstOffset))
go(3, 0)
firstOffset = turn(45)
go(0.7,0)
secondError = turn(abs(45 + firstOffset))
go(4.5, 0)
firstOffset = turn(22)
go(2.061,0)
secondError = turn(abs(78 + firstOffset))
firstOffset = turn(45)
go(0.7,0)
secondError = turn(abs(45 + firstOffset))
go(1.5,0)
firstOffset = turn(45)
go(0.7,0)
secondError = turn(abs(45 + firstOffset))
dropOff()