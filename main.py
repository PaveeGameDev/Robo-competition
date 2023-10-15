#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
line_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=162)

# constanc
BLACK = 17
WHITE = 57
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 50
PROPORTIONAL_GAIN = 8

# Start following the line endlessly.
while True:
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = deviation * abs(deviation) / 3
    print(turn_rate)

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)