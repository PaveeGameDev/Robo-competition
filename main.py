#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch
from points import points, ordered_points
import time
import math

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
# grabber_motor = Motor(Port.C)
line_sensor = ColorSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)
ev3 = EV3Brick()
stop_watch = StopWatch()

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
DROP_OFF_SPEED = 100
TIME_TO_MIDDLE = 80 * 1000
TIME_TO_STOP = 83 * 1000
DISTANCE_MULTIPLIER = 280


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
    print("getting cover")
    while true:
        left_motor.run(-DRIVE_SPEED)
        right_motor.run(-DRIVE_SPEED)


def dropOff():
    print("dropping off")
    grabber_motor.run_time(DROP_OFF_SPEED, 5 * 1000, then=Stop.HOLD, wait=True)
    getCover()


def needToGoMiddle():
    global wentToMiddle
    wentToMiddle = True
    back = DPS.trajectory(points[ordered_points[-1]][0], points[ordered_points[-1]][1])
    while not back:
        back = DPS.trajectory(
            points[ordered_points[-1]][0], points[ordered_points[-1]][1]
        )
    dropOff()


##Math functions
def mod(a, b):
    if a < 0 and b > 0 or a > 0 and b < 0:
        return a % -b
    else:
        return a % b


##ROBO DPS (- Davis Positioning System)
class DPS_class:
    def __init__(self, x, y):  # distance in mm, time in s, angle in degrees
        self.time = time.time()
        self.angle = 0
        self.zelta = -90
        self.x = x
        self.y = y
        self.speed = 0
        self.turning = 0
        self.gyro = 0

    def calc(self, speed, turning_rate):
        deltaT = time.time() - self.time
        self.time = time.time()
        robot.drive(speed, turning_rate)
        gyro1 = gyro_sensor.angle()

        beta = self.turning * deltaT
        difference = self.gyro - (self.angle + beta)
        if abs(difference) >= 2:
            print("fixing " + str(difference))
            beta += difference

        if self.turning != 0:
            radius = (180 * self.speed) / (math.pi * self.turning)
            gama = (180 - beta) / 2
            b = 2 * math.sin((beta / 2) * DEG_TO_RAD) * radius
            self.x += b * math.cos((gama - self.zelta) * DEG_TO_RAD)
            self.y += b * math.sin((gama - self.zelta) * DEG_TO_RAD)
            self.zelta += beta
            self.angle = self.zelta + 90
        else:  ##robot jede po rovny primce, ktera je pod uhlem self.angle
            self.x += math.cos(self.angle * DEG_TO_RAD) * self.speed * deltaT
            self.y += math.sin(self.angle * DEG_TO_RAD) * self.speed * deltaT

        self.gyro = gyro1
        self.speed = speed
        self.turning = turning_rate

    # -#-#-#-#-#-#-#-#-#-#-#-#-#-#-
    # --#-#-#-#-#-#-#-#-#-#-#-#-#-#
    def pairAngle(self, a):
        if a < 0:
            return a + 360
        else:
            return a - 360

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
        if deltaX < 0:
            alpha += 180

        if alpha + 2 >= mod(self.angle, 360) and alpha - 2 <= mod(self.angle, 360):
            print("    driving")
            print(alpha, deltaX, deltaY)
            print(self.angle, self.x, self.y)
            self.calc(speed=200, turning_rate=0)
        elif self.pairAngle(alpha + 2) >= mod(self.angle, 360) and self.pairAngle(
            alpha - 2
        ) <= mod(self.angle, 360):
            print("    driving2")
            print(alpha, deltaX, deltaY)
            print(self.angle, self.x, self.y)
            self.calc(speed=200, turning_rate=0)
        else:
            deltaAngle = alpha - mod(self.angle, 360)
            print("    turning")
            print(alpha)
            print(self.angle)
            self.calc(
                speed=0, turning_rate=deltaAngle * 2
            )  # ten nasobitel se bude menit - musi se najit nelepsi hodnota

        if self.x + 10 > x and self.x - 10 < x and self.y + 10 > y and self.y - 10 < y:
            return True
        else:
            return False


# Start following the line endlessly.
DPS = DPS_class(0, 0)
check = False
print(DPS.x, DPS.y, DPS.zelta, DPS.angle)
destination = [points[ordered_points[0]][0], points[ordered_points[0]][1]]
i = 0
while True:
    current_time = time.time()
    current_time_from_start = current_time - START_TIME

    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = deviation * abs(deviation) / TURN_RATE_DIVIDER * turn_rate_multiplier

    # Updates robot positioning system and tells robot to drive

    back = DPS.trajectory(destination[0], destination[1])

    if back:
        if current_time_from_start > TIME_TO_MIDDLE and not wentToMiddle:
            needToGoMiddle()
        i += 1

        if i < len(points):
            print("#####--NEW DESTINATION--#####")
            destination = [
                points[ordered_points[i]][0],
                points[ordered_points[i]][1],
            ]
            
        else:
            #dropOff()
            robot.stop()
            ev3.speaker.beep()
            print(DPS.x, DPS.y, DPS.zelta, DPS.angle)
            break

    # grabber_motor.run(GRAB_SPEED)

    if current_time_from_start > TIME_TO_STOP and gettingCover:
        ev3.speaker.beep()
        break
