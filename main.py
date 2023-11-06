#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
import time
import math

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
#grabber_motor = Motor(Port.C)
line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
ev3 = EV3Brick()


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
    #TODO: implement
    

def dropOff():
    print('dropping off')
    time.sleep(10)
    getCover()

def needToGoMiddle():
    global wentToMiddle
    wentToMiddle = True
    #TODO: implement
    dropOff()

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

    def calc(self, speed, turning_rate):
        deltaT = time.time() - self.time
        self.time = time.time()
        robot.drive(speed, turning_rate)

        beta = self.turning * deltaT
        difference = gyro_sensor.angle() - (self.angle + beta)
        if (abs(difference) >= 2):
            print('fixing')
            beta += difference
        
        if self.turning != 0:
            radius = (180 * self.speed) / (math.pi * self.turning)
            gama = (180 - beta) / 2
            b = 2 * math.sin((beta/2) * DEG_TO_RAD) * radius
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

        if alpha + 2 >= self.angle % 360 and alpha - 2 <= self.angle % 360:
            print("driving")
            print(alpha,deltaY,deltaX)
            self.calc(speed = 200, turning_rate = 0)
        else:
            deltaAngle = alpha - (self.angle % 360)
            print("turning")
            print(alpha,deltaY,deltaX)
            self.calc(speed = 0, turning_rate = deltaAngle * 2) #ten nasobitel se bude menit - musi se najit nelepsi hodnota

        if self.x + 5 > x and self.x - 5 < x and self.y + 5 > y and self.y - 5 < y:
            return True
        else:
            return False

# Start following the line endlessly.
DPS = DPS_class(0, 0)
#rate = -22.5
check = False
print(DPS.x,DPS.y,DPS.zelta,DPS.angle)

while True:
    current_time = time.time()
    current_time_from_start = current_time - START_TIME

    """ if not check and gyro_sensor.angle() < -90:
        check = True
        rate = 0
        Y = DPS.y
        print(DPS.x,DPS.y,DPS.zelta,DPS.angle)
    
    if check and Y * 2 > DPS.y :
        robot.stop()
        ev3.speaker.beep()
        print(DPS.x,DPS.y,DPS.zelta,DPS.angle)
        break """

    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = deviation * abs(deviation) / TURN_RATE_DIVIDER * turn_rate_multiplier

    # Updates robot positioning system and tells robot to drive
    #DPS.calc(DRIVE_SPEED, turn_rate)
    back = DPS.trajectory(0, 1000)

    if back:
        robot.stop()
        ev3.speaker.beep()
        print(DPS.x,DPS.y,DPS.zelta,DPS.angle)
        break

    """ grabber_motor.run(GRAB_SPEED)
    
    if current_time_from_start > 10 and not wentToMiddle:
        needToGoMiddle()
        
    if current_time_from_start > 83 and gettingCover:
        ev3.speaker.beep()
        break """