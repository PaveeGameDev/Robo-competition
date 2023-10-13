#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    InfraredSensor,
    UltrasonicSensor,
    GyroSensor,
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

# #!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import Motor
# from pybricks.parameters import Port

# # Create your objects here

# # Initialize the EV3 Brick.
# ev3 = EV3Brick()

# # Initialize a motor at port B.
# test_motor = Motor(Port.B)

# # Write your program here

# # Play a sound.
# ev3.speaker.beep()

# # Run the motor up to 500 degrees per second. To a target angle of 90 degrees.
# test_motor.run_target(500, 90)

# # Play another beep sound.
# ev3.speaker.beep(frequency=1000, duration=500)


# #!/usr/bin/env pybricks-micropython

# """
# Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
# ----------------------------------------------------------------------

# This program requires LEGO® EV3 MicroPython v2.0.
# Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

# Building instructions can be found at:
# https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
# """

# from pybricks.ev3devices import Motor, ColorSensor
# from pybricks.parameters import Port
# from pybricks.tools import wait
# from pybricks.robotics import DriveBase

# # Initialize the motors.
# left_motor = Motor(Port.B)
# right_motor = Motor(Port.C)

# # Initialize the color sensor.
# line_sensor = ColorSensor(Port.S3)

# # Initialize the drive base.
# robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# # Calculate the light threshold. Choose values based on your measurements.
# BLACK = 9
# WHITE = 85
# threshold = (BLACK + WHITE) / 2

# # Set the drive speed at 100 millimeters per second.
# DRIVE_SPEED = 100

# # Set the gain of the proportional line controller. This means that for every
# # percentage point of light deviating from the threshold, we set the turn
# # rate of the drivebase to 1.2 degrees per second.

# # For example, if the light value deviates from the threshold by 10, the robot
# # steers at 10*1.2 = 12 degrees per second.
# PROPORTIONAL_GAIN = 1.2

# # Start following the line endlessly.
# while True:
#     # Calculate the deviation from the threshold.
#     deviation = line_sensor.reflection() - threshold

#     # Calculate the turn rate.
#     turn_rate = PROPORTIONAL_GAIN * deviation

#     # Set the drive base speed and turn rate.
#     robot.drive(DRIVE_SPEED, turn_rate)

#     # You can wait for a short time or do other things in this loop.
#     wait(10)
