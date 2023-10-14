import movement

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.hubs import EV3Brick

# Initialize the motors and sensors.
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)
lineSensor = ColorSensor(Port.S3)

ev3 = EV3Brick()

while True:
    movement.moveByLine(leftMotor, rightMotor, lineSensor)
    if ev3.buttons.pressed().length > 0:
        movement.changeLine()
    wait(10)