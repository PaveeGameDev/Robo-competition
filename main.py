#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
import time
import math

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
ev3 = EV3Brick()


# constants
DEG_TO_RAD = math.pi/180
BLACK = 5
WHITE = 30
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 10
TURN_RATE_DIVIDER = 3
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 150
START_TIME = time.time()


# second initialization
robot = DriveBase(
    left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK
)

current_time = time.time()
current_time_from_start = 0
last_gyro_check_time = current_time
gyro_angle = 0
turn_list = []
can_go_middle = 0
turn_rate_multyplier = 1
going_middle = 0


def writeTurn(isLeft):
    turn_list.append(isLeft)
    print(turn_list)


def checkGyroMovements():
    if gyro_sensor.angle() > 85:
        writeTurn(0)
        gyro_sensor.reset_angle(0)
    elif gyro_sensor.angle() < -85:
        writeTurn(1)
        gyro_sensor.reset_angle(0)


def goMiddle():
    global can_go_middle
    global robot
    global turn_rate_multyplier
    global going_middle
    print("going middle")
    robot.stop()
    robot.straight(100)
    robot.turn(50)
    turn_rate_multyplier = -1
    can_go_middle = 0
    going_middle = 0


def tryGoMiddle():
    global going_middle
    last_three_items = turn_list[-3:]
    if last_three_items == [0, 0, 1]:
        going_middle = 1


def calculateGoMiddle():
    global can_go_middle
    last_three_items = turn_list[-3:]

    if last_three_items == [1, 0, 1]:
        can_go_middle = 1

##ROBO DPS (- Davis Positioning System)
class DPS_class:
  def __init__(self, x, y):
    self.time = time.time()
    self.angle = 0
    self.x = 0
    self.y = 0

    def calc(self, speed, turning_rate):
        deltaT = self.time - time.time()
        if turning_rate != 0:
            radius = (180 * speed)/(math.pi * turning_rate)
            alpha = turning_rate * deltaT

            X = math.sin(alpha * DEG_TO_RAD) * radius
            if 90 < turning_rate * deltaT and turning_rate * deltaT < 270
                Y = radius + math.cos(alpha * DEG_TO_RAD) * radius
            else:
                Y = radius - math.cos(alpha * DEG_TO_RAD) * radius
            ##stred kruznice po ktere robot jede neni v bode [0,0] ale je o polomer ve smeru Y posunuty

            if self.angle != 0:
                radius2 = math.sqrt(Y**2 + X**2)
                self.angle += math.asin(Y/radius2)/DEG_TO_RAD
                self.x += math.cos(self.angle * DEG_TO_RAD) * radius2
                self.y += math.sin(self.angle * DEG_TO_RAD) * radius2
            else:
                self.angle += alpha
                self.x += X
                self.y += Y
            self.time = time.time()
        else:##robot jede po rovny primce, ktera je pod uhlem self.angle
            self.x += math.cos(self.angle * DEG_TO_RAD) * (speed * deltaT)
            self.y += math.sin(self.angle * DEG_TO_RAD) * (speed * deltaT)
            self.time = time.time()



# Start following the line endlessly.
DPS = DPS_class(0,0)
while True:
    current_time = time.time()
    current_time_from_start = current_time - START_TIME
    checkGyroMovements()

    if current_time_from_start > 80:
        ev3.speaker.beep()
        break

    if current_time_from_start > 70 and going_middle == 1:
        goMiddle()

    calculateGoMiddle()
    if can_go_middle == 1:
        tryGoMiddle()

    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = deviation * abs(deviation) / TURN_RATE_DIVIDER * turn_rate_multyplier
    # print("turn rate: " + str(turn_rate))

    #updates robot positioning system !!!!.NEEDS to be JUST BEFORE .drive()!!!
    DPS.calc(DRIVE_SPEED, turn_rate)
    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)
