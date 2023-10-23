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


def rightTurn():
    print("right")
    gyro_sensor.reset_angle(0)


def leftTurn():
    print("left")
    gyro_sensor.reset_angle(0)


def checkGyroMovements():
    if gyro_sensor.angle() > 80:
        rightTurn()
    elif gyro_sensor.angle() < -80:
        leftTurn()


# Start following the line endlessly.
while True:
    current_time = time.time()
    current_time_from_start = current_time - START_TIME
    checkGyroMovements()

    print(gyro_sensor.angle())

    if current_time_from_start > 80:
        ev3.speaker.beep()
        break

    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = deviation * abs(deviation) / TURN_RATE_DIVIDER
    # print("turn rate: " + str(turn_rate))

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    wait(1000)
