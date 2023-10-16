#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import threading 

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
line_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=162)

# constanc
BLACK = 17
WHITE = 57
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 150
TURN_RATE_DIVIDER = 3

threading.Timer(45,firstTreshold).start()


def firstTreshold:
    print("Half time")
    threading.Timer(25,secondTreshold).start()

def secondTreshold:
    print("70 seconds")
    threading.Timer(10,thirdTreshold).start()

def thirdTreshold:
    print("80 seconds")
    threading.Timer(3,fourthTreshold).start()

def fourthTreshold:
    print("83 seconds")
    threading.Timer(7,fifthTreshold).start()

def fifthTreshold:
    print("Time is up")

# Start following the line endlessly.
while True:
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = deviation * abs(deviation) / TURN_RATE_DIVIDER
    print(turn_rate)

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)
