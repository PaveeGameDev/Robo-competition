#!/usr/bin/env pybricks-micropython
from importing import robot, gyro_sensor, USsensor, ev3, stop_watch, #grabber_motor, line_sensor
import time
import math


DEG_TO_RAD = math.pi / 180
##Math functions
def mod(a, b):
    if a < 0 and b > 0 or a > 0 and b < 0:
        return a % -b
    else:
        return a % b


##ROBO DPS (- Davis Positioning System)
class DPS_class:
    def __init__(self, x, y, angle):  # distance in mm, time in s, angle in degrees
        self.time = time.time()
        self.angle = angle
        self.zelta = -90 + angle
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
        elif (self.pairAngle(alpha + 2) >= mod(self.angle, 360)
             and self.pairAngle(alpha - 2) <= mod(self.angle, 360)):
            print("    driving2")
            print(alpha, deltaX, deltaY)
            print(self.angle, self.x, self.y)

            self.calc(speed=200, turning_rate=0)
        else:
            deltaAngle = alpha - mod(self.angle, 360)
            print("    turning")
            print(alpha)
            print(self.angle)

            self.calc(speed=0, turning_rate=deltaAngle * 2)

        if self.x + 10 > x and self.x - 10 < x and self.y + 10 > y and self.y - 10 < y:
            return True
        else:
            return False
    

    def turn(self, angle):
        beggining_angle = self.angle
        while True:
            deltaAngle = mod(self.angle, 360) - angle
            self.calc(speed = 0, turning_rate = deltaAngle)

            if self.angle + 2 >= beggining_angle + angle and self.angle - 2 <= beggining_angle + angle:
                break
        
    def go_by_wall(self, path, distance, side, offset): ##side (1 or -1) is the side where ultrasonic sensor is (negatve = left, positive = right)
        beggining = [self.x, self.y]                 ##offset negative is for left side positive for right side
        end = [self.x + path[0], self.y + path[1]]
        right = False
        left = False

        while True:
            print("wall loop")
            real_distance = USsensor.distance() + offset[0]*side
            
            print(real_distance)
            
            if real_distance + 15 >= distance and not left:
                right = False
                print("1 turn")
                self.calc(speed = 100, turning_rate = -100*side)
            elif real_distance - 15 <= distance and not right:
                left = False
                print("else")
                self.calc(speed = 100, turning_rate = 100*side)
            else:
                print("strait")
                self.calc(speed = 100, turning_rate = 0)

            if path[1] == 0:
                if path[0] > 0:
                    if self.x >= end[0]:
                        print('1')
                        return True
                elif path[0] < 0:
                    if self.x <= end[0]:
                        print('2')
                        return True
            elif path[0] == 0:
                if path[1] > 0:
                    if self.y >= end[1]:
                        print('3')
                        return True
                elif path[1] < 0:
                    if self.y <= end[1]:
                        print('4')
                        return True
