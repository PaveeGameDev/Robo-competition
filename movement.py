BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 100
PROPORTIONAL_GAIN = 1.2
isLeft = True

def drive(robot, speed, turnRate):
    robot.drive(speed, turnRate)


def moveByLine(leftMotor, rightMotor, lineSensor):

    deviation = lineSensor.reflection() - threshold
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot = DriveBase(leftMotor, rightMotor, wheelDiameter=55.5, axleTrack=104)
    drive(robot, DRIVE_SPEED, turn_rate)


def changeLine():
    global isLeft
    isLeft = not isLeft
    print (isLeft)
