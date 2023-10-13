import movement

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the motors and sensors.
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)
lineSensor = ColorSensor(Port.S3)

while True:
    movement.moveByLine(leftMotor, rightMotor, lineSensor)
    wait(10)
