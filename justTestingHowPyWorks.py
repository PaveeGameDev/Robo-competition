self.distance = [] #
self.lostAngle = 0
LostTheTarget = False
USangle = 5
RightSide = False


distance = Usensor.distance()
if not LostTheTarget:
    self.distance.insert(0, distance)
    if len(self.distance) > 5:
        self.distance.pop()

average = sum(self.distance) / len(self.distance)

z = math.sqrt(deltaX**2 + deltaY**2)

if z < 500 and not LostTheTarget and distance + 5 < average or distance - 5 > average:
    print("lost the target")
    LostTheTarget = True
    self.lostAngle = Self.Angle

if LostTheTarget and distance + 10 > average and distance - 10 < average:
    LostTheTarget = False
    newX = math.sin(self.angle * DEG_TO_RAD) * distance
    newY = math.cos(self.angle * DEG_TO_RAD) * distance
    points[i][0] = newX
    points[i][1] = newY
    destination[0] = newX
    destination[1] = newY

if self.angle + USangle - 1 < Self.lostAngle:
    if not RightSide:
        RightSide = True
        ##nejak switchnout otačeni

if self.angle - 2*USangle - 1 < Self.lostAngle:
    if LostTheTarget:
        print("kostka ztracena")

if LostTheTarget:
    deltaAngle = self.angle - self.lostAngle
    self.calc(speed = 0, turning_rate = deltaAngle * 2)