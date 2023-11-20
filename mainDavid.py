#!/usr/bin/env pybricks-micropython
from importing import robot,line_sensor,gyro_sensor,USsensor,ev3,stop_watch #, grabber_motor
from DPS import DPS_class
import time
import math

# constants
DEG_TO_RAD = math.pi / 180
BLACK = 15 ## GET RIGHT VALUE
WHITE = 40 ## GET RIGHT VALUE
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 100
START_TIME = time.time()
GRAB_SPEED = 500
DROP_OFF_SPEED = 100
TIME_TO_MIDDLE = 80 * 1000
TIME_TO_STOP = 83 * 1000
DISTANCE_MULTIPLIER = 280

DPS = DPS_class(70, 1540,0)
list_of_coordinates = [
    [2240, 140],
    [70, 140],
    [70, 1540],
    [840, 1080]]

while True:
    return_value = DPS.trajectory( 2240, 1540)

    if return_value:
        X_value = list_of_coordinates[0][0]
        Y_value = list_of_coordinates[0][1]
        list_of_coordinates.pop(0)