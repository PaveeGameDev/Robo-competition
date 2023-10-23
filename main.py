#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
import time

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
ev3 = EV3Brick()


# constants
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

##ROBO DPS (- Davis Positioning System) WIP
class DPS_class:
  def __init__(self, x, y):
    self.time = time.time()
    self.x = 0
    self.y = 0

    def calc(self, speed, turning_rate):
        deltaT = self.time - time.time()
        radius = (180 * speed)/(math.pi * turning_rate)
        #x += 


# Start following the line endlessly.
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

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)
