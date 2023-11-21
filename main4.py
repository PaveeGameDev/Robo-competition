#!/usr/bin/env pybricks-micropython
from importing import robot,  ev3, stop_watch,gyro_sensor #grabber_motor, line_sensor , USsensor,
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