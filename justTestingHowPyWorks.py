self.distance = [] #
self.lostAngle = 0
LostTheTarget = False
RightSide = False
sensorDelta = [x,y]
USangle = 5


distance = Usensor.distance()
if not LostTheTarget: ##if the robot hadn't lost the cube, save the distance from cube and upadt avarage
    self.distance.insert(0, distance)
    if len(self.distance) > 5:
        self.distance.pop()
    average = sum(self.distance) / len(self.distance)

z = math.sqrt((deltaX - sensorDelta[0])**2 + (deltaY - sensorDelta[1])**2)

if z < 500 and not LostTheTarget and distance + 5 < average or distance - 5 > average and alpha + 2 >= mod(self.angle, 360) and alpha - 2 <= mod(self.angle, 360): ##testing if sensor lost the object i.e. its really far from average
    print("lost the target")
    LostTheTarget = True
    self.lostAngle = Self.Angle
elif z < 500 and not LostTheTarget and distance + 5 < average or distance - 5 > average and self.pairAngle(alpha + 2) >= mod(self.angle, 360) and self.pairAngle(alpha - 2) <= mod(self.angle, 360):
    print("lost the target")
    LostTheTarget = True
    self.lostAngle = Self.Angle
if LostTheTarget and distance + 10 > average and distance - 10 < average: ##tesing if the lost object was found i.e. its cloase to the last avarage
    LostTheTarget = False
    newX = math.sin(self.angle * DEG_TO_RAD) * distance ## now i know more precisely the angle where the object is, so i can calculate new position 
    newY = math.cos(self.angle * DEG_TO_RAD) * distance
    points[i][0] = newX
    points[i][1] = newY
    destination[0] = newX
    destination[1] = newY

if self.angle + USangle - 1 < Self.lostAngle: ##testing if the robot turned the wanted amount of degrees to side, so it can start turning to the second one
    if not RightSide:
        RightSide = True
        ##nejak switchnout otačeni

if self.angle - 2*USangle - 1 < Self.lostAngle: ##testing if robot turned the wanted amount of degrees to second side
    if LostTheTarget: ##if the cude still isnt found than there is sth. wronr
        print("kostka ztracena")

if LostTheTarget: ##the moving part of finding lost object
    deltaAngle = self.angle - self.lostAngle
    self.calc(speed = 0, turning_rate = deltaAngle * 2)