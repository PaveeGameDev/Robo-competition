#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch
import time
import math

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
#grabber_motor = Motor(Port.C)
line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
ev3 = EV3Brick()
stop_watch = StopWatch()

# constants
DEG_TO_RAD = math.pi / 180
BLACK = 15
WHITE = 40
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 100
TURN_RATE_DIVIDER = 3
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 150
START_TIME = time.time()
GRAB_SPEED = 500
DROP_OFF_SPEED = 100
TIME_TO_MIDDLE = 10 * 1000
TIME_TO_STOP = 83 * 1000
sensorDelta = [x,y]
USangle = 5


# second initialization
robot = DriveBase(
    left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
)

current_time = time.time()
current_time_from_start = 0
gyro_angle = 0
turn_rate_multiplier = 1
wentToMiddle = False
gettingCover = False


def getCover():
    global gettingCover
    gettingCover = True
    print("getting cover")
    # TODO: implement


def dropOff():
    print("dropping off")
    grabber_motor.run_time(DROP_OFF_SPEED, 10 * 1000, then=Stop.HOLD, wait=True)
    getCover()


def needToGoMiddle():
    global wentToMiddle
    wentToMiddle = True
    # TODO: implement
    dropOff()

##Math functions
def mod(a,b):
    if a < 0 and b > 0 or a > 0 and b < 0:
        return a % -b
    else:
        return a % b
##ROBO DPS (- Davis Positioning System)
class DPS_class:
    def __init__(self, x, y):#distance in mm, time in s, angle in degrees
        self.time = time.time()
        self.angle = 0
        self.zelta = -90
        self.x = 0
        self.y = 0
        self.speed = 0
        self.turning = 0
        self.distance = [] ##aray of last 5 distances of the object
        self.lostAngle = 0

    def calc(self, speed, turning_rate):
        deltaT = time.time() - self.time
        self.time = time.time()
        robot.drive(speed, turning_rate)
        print("time " + str(deltaT))

        beta = self.turning * deltaT
        difference = gyro_sensor.angle() - (self.angle + beta)
        if (abs(difference) >= 2):
            print('fixing ' + str(difference))
            beta += difference

        if self.turning != 0:
            radius = (180 * self.speed) / (math.pi * self.turning)
            gama = (180 - beta) / 2
            b = 2 * math.sin((beta / 2) * DEG_TO_RAD) * radius
            self.x += b * math.cos((gama - self.zelta) * DEG_TO_RAD)
            self.y += b * math.sin((gama - self.zelta) * DEG_TO_RAD)
            self.zelta += beta
            self.angle = self.zelta + 90
        else:  ##robot jede po rovny primce, ktera je pod uhlem self.angle
            self.x += math.cos(self.angle * DEG_TO_RAD) * self.speed * deltaT
            self.y += math.sin(self.angle * DEG_TO_RAD) * self.speed * deltaT

        self.speed = speed
        self.turning = turning_rate
    
    #-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
    #--#-#-#-#-#-#-#-#-#-#-#-#-#-#
    def pairAngle(self,a):
        if a < 0:
            return a + 360
        else:
            return a - 360

    def trajectory(self, x, y):
        deltaX = x - self.x
        deltaY = y - self.y
        if deltaX == 0:
            if deltaY > 0:
                alpha = 90
            else:
                alpha = -90
        else:
            m = deltaY / deltaX
            alpha = math.atan(m) / DEG_TO_RAD
        if deltaX < 0:
            alpha += 180

        ##US sensor object detection and finding
        distance = Usensor.distance()
        if not LostTheTarget: ##if the robot hadn't lost the cube, save the distance from cube and upadt avarage
            self.distance.insert(0, distance)
            if len(self.distance) > 5:
                self.distance.pop()
            average = sum(self.distance) / len(self.distance)

        z = math.sqrt((deltaX - sensorDelta[0])**2 + (deltaY - sensorDelta[1])**2)

        if z < 500 and not LostTheTarget and distance + 5 < average or distance - 5 > average and alpha + 2 >= mod(self.angle, 360) and alpha - 2 <= mod(self.angle, 360): ##testing if sensor lost the object i.e. its really far from average
            print("lost the target")
            LostTheTarget = True
            self.lostAngle = Self.Angle
        elif z < 500 and not LostTheTarget and distance + 5 < average or distance - 5 > average and self.pairAngle(alpha + 2) >= mod(self.angle, 360) and self.pairAngle(alpha - 2) <= mod(self.angle, 360):
            print("lost the target")
            LostTheTarget = True
            self.lostAngle = Self.Angle
        if LostTheTarget and distance + 10 > average and distance - 10 < average: ##tesing if the lost object was found i.e. its cloase to the last avarage
            LostTheTarget = False
            newX = math.sin(self.angle * DEG_TO_RAD) * distance ## now i know more precisely the angle where the object is, so i can calculate new position 
            newY = math.cos(self.angle * DEG_TO_RAD) * distance
            points[i][0] = newX
            points[i][1] = newY
            destination[0] = newX
            destination[1] = newY

        if self.angle + USangle - 1 < Self.lostAngle: ##testing if the robot turned the wanted amount of degrees to side, so it can start turning to the second one
            if not RightSide:
                RightSide = True
                ##nejak switchnout otačeni

        if self.angle - 2*USangle - 1 < Self.lostAngle: ##testing if robot turned the wanted amount of degrees to second side
            if LostTheTarget: ##if the cude still isnt found than there is sth. wronr
                print("kostka ztracena")

        if LostTheTarget: ##the moving part of finding lost object
            deltaAngle = self.angle - self.lostAngle
            self.calc(speed = 0, turning_rate = deltaAngle * 2)
            #--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#
            #--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#--#
        elif alpha + 2 >= mod(self.angle, 360) and alpha - 2 <= mod(self.angle, 360): ##movement if there is no problem with object
            print("    driving")
            print(alpha,deltaX,deltaY)
            print(self.angle,self.x,self.y)
            self.calc(speed = 200, turning_rate = 0)
        elif self.pairAngle(alpha + 2) >= mod(self.angle, 360) and self.pairAngle(alpha - 2) <= mod(self.angle, 360):
            print("    driving2")
            print(alpha,deltaX,deltaY)
            print(self.angle,self.x,self.y)
            self.calc(speed = 200, turning_rate = 0)
        else:
            deltaAngle = alpha - mod(self.angle, 360)
            print("    turning")
            print(alpha,deltaX,deltaY)
            print(self.angle,self.x,self.y)
            print(deltaAngle)
            print(mod(self.angle, 360))
            self.calc(speed = 0, turning_rate = deltaAngle * 2) #ten nasobitel se bude menit - musi se najit nelepsi hodnota

        if self.x + 5 > x and self.x - 5 < x and self.y + 5 > y and self.y - 5 < y:
            return True
        else:
            return False

# Start following the line endlessly.
DPS = DPS_class(0, 0)
print(DPS.x, DPS.y, DPS.zelta, DPS.angle)
destination = [points[0][0],points[0][1]]
i = 0

check = False
LostTheTarget = False
RightSide = False

while True:
    current_time = time.time()
    current_time_from_start = current_time - START_TIME

    """ if not check and gyro_sensor.angle() < -90:
        check = True
        rate = 0
        Y = DPS.y
        print(DPS.x, DPS.y, DPS.zelta, DPS.angle)

    if check and Y * 2 > DPS.y:
        robot.stop()
        ev3.speaker.beep()
        print(DPS.x,DPS.y,DPS.zelta,DPS.angle)
        break """

    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = deviation * abs(deviation) / TURN_RATE_DIVIDER * turn_rate_multiplier

    # Updates robot positioning system and tells robot to drive
    #DPS.calc(DRIVE_SPEED, turn_rate)
    back = DPS.trajectory(destination[0],destination[1])

    if back:
        i += 1
        if i < len(points):
            destination = [points[i][0],points[i][1]]
        else:
            robot.stop()
            ev3.speaker.beep()
            print(DPS.x,DPS.y,DPS.zelta,DPS.angle)
            break

    """ grabber_motor.run(GRAB_SPEED)
    
    if current_time_from_start > 10 and not wentToMiddle:
        needToGoMiddle()

    if stop_watch.time > TIME_TO_STOP and gettingCover:
        ev3.speaker.beep()
        break """
