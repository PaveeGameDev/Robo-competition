from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor, ColorSensor

BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 100
PROPORTIONAL_GAIN = 1.2
isLeft = True


def moveByLine(leftMotor, rightMotor, lineSensor):
    multiplier = -1
    if isLeft:
        multiplier = 1
    deviation = lineSensor.reflection() - threshold
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot = DriveBase(leftMotor, rightMotor, wheel_diameter=55.5, axle_track=162)
    robot.drive(DRIVE_SPEED, turn_rate)


def changeLine():
    global isLeft
    isLeft = not isLeft
    print("Changing lines. Am I going left? :" + isLeft)
