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
CUBE_PICKUP_ORDER = [
 True,  False, False, False, True,  False, False, False, True, False, False, False, True,
 False, False, True,  False, False, True,  False, False, True, False, False, True,
 False, True,  False, True,  False, True,  False, True,
 True,  True,  True,  True]
SIDES_OF_MAP = [
 "a","b","c","d",
 "a","b","c","d",
 "a","b","c","d",
 "a","b","c","d"
]
END_OF_SIDES = {
 "a" : [1960,420],
 "b" : [1960,1260],
 "c" : [560,1260],
 "d" : [560,420]
}
LINE_TO_CUBE = 280 # lengh in mm

def pick_cube():
    print("hello, world")
    ev3.speaker.beep()
    """ begginnig = [DPS.x,DPS.y]
    DPS.turn(-90)
    headingY = ((self.angle) / 90) % 2
    #TODO: let grabber motot run
    while True:
        if abs(deviation) > WHITE/4 + 3*BLACK/4 and abs(deviation) < 3*WHITE/4 + BLACK/4:
            DPS.calc(300, 0)
        else:
            DPS.calc(60, deviation**3)
        if headingY:
            if beggining[1] + LINE_TO_CUBE < DPS.y or beggining[1] - LINE_TO_CUBE > DPS.y:
                DPS.turn(180)
                break
        else:
            if beggining[0] + LINE_TO_CUBE < DPS.x or beggining[0] - LINE_TO_CUBE > DPS.x:
                DPS.turn(180)
                break
    
    while True:
        if abs(deviation) > WHITE/4 + 3*BLACK/4 and abs(deviation) < 3*WHITE/4 + BLACK/4:
            DPS.calc(300, 0)
        else:
            DPS.calc(60, deviation**3)
        if headingY:
            if beggining[1] + 10 > DPS.y or beggining[1] - 10 < DPS.y:
                DPS.turn(-90)
                break
        else:
            if beggining[0] + 10 > DPS.x or beggining[0] - 10 < DPS.x:
                DPS.turn(-90)
                break
     """
    go_pick_cube = False
    #TODO: stop grabber motot

cube = False
go_pick_cube = False
DPS = DPS_class(0,0,0)
list_of_distances = []
average = USsensor.distance()
turn_off = False ## variable for stopping robot from looking for cubes

while True:
    distance = USsensor.distance()

    
    if (distance > average + 20 or distance < average - 20) and not cube and not turn_off:
        cube = True

    if not cube:
        list_of_distances.append(distance)
        if len(list_of_distances) > 5:
            list_of_distances.pop(0)
    average = sum(list_of_distances) / len(list_of_distances)
        
    if cube:
        if CUBE_PICKUP_ORDER[0]:
            go_pick_cube = True
        CUBE_PICKUP_ORDER.pop(0)
        cube = False

    deviation = line_sensor.reflection() - threshold

    if not go_pick_cube:
        #if abs(deviation) > WHITE/4 + 3*BLACK/4 and abs(deviation) < 3*WHITE/4 + BLACK/4:
        DPS.calc(300, 0)
        #else:
        #    DPS.calc(60, deviation**3)
        #DONE: go strait
        #DONE: use DPS to know where aproximetly is robot is
        #DONE: use line
    else:
        pick_cube()
    
    #TODO-OPTIONAL
    # when robot is on line, reset one of coordinates to coorditanes of the line

    if SIDES_OF_MAP[0] == "a" and END_OF_SIDES[SIDES_OF_MAP[0]][0] < DPS.x:
        SIDES_OF_MAP.pop(0)
        DPS.turn(90)
    elif SIDES_OF_MAP[0] == "b" and END_OF_SIDES[SIDES_OF_MAP[0]][1] < DPS.y:
        SIDES_OF_MAP.pop(0)
        DPS.turn(90)
    elif SIDES_OF_MAP[0] == "c" and END_OF_SIDES[SIDES_OF_MAP[0]][0] > DPS.x:
        SIDES_OF_MAP.pop(0)
        DPS.turn(90)
    elif SIDES_OF_MAP[0] == "d" and END_OF_SIDES[SIDES_OF_MAP[0]][1] > DPS.y:
        SIDES_OF_MAP.pop(0)
        DPS.turn(90)


