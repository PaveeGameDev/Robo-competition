#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch
import time
import math

# Initialize everything
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
grabber_motor = Motor(Port.C)
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
TIME_TO_MIDDLE = 10 * 1000
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
current_point = 0


def getCover():
    global gettingCover
    gettingCover = True
    print("getting cover")
    # TODO: implement david


def dropOff():
    print("dropping off")
    grabber_motor.run_time(DROP_OFF_SPEED, 10 * 1000, then=Stop.HOLD, wait=True)
    getCover()


def needToGoMiddle():
    global wentToMiddle
    wentToMiddle = True
    # TODO: implement david
    dropOff()


def find_fastest_routes(points, current_point, target_point):
    def dfs(current, path, time):
        if current == target_point:
            return [(path, time)]
        if current < 0 or current >= len(points):
            return []

        fastest_routes = []

        for i, (next_point, travel_time) in enumerate(points[current]):
            if next_point not in path:
                new_path = path + [next_point]
                new_time = time + travel_time

                routes = dfs(next_point, new_path, new_time)
                fastest_routes.extend(routes)

        min_time = float("inf")
        for route, time in fastest_routes:
            if time < min_time:
                min_time = time

        return [(route, time) for route, time in fastest_routes if time == min_time]

    initial_path = [current_point]
    routes = dfs(current_point, initial_path, 0)
    return [route for route, _ in routes]


# Define your array of points
points = [
    [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
    [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
    [[0, 1], [7, 1], [9, 1], [8, 1]],
    [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
    [[5, 1], [6, 1], [7, 1.5], [1, 1]],
    [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
    [[11, 1], [10, 1], [12, 1.5], [4, 1]],
    [[4, 1.5], [12, 1], [18, 1.5], [2, 1]],
    [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
    [[2, 1], [-1, 0], [14, 0.5], [13, 1]],
    [[15, 1], [16, 1], [17, 1.5], [6, 1]],
    [[-1, 0], [-1, 0], [6, 1], [-1, 0]],
    [[6, 1.5], [17, 1], [20, 1.5], [7, 1]],
    [[-1, 0], [9, 1], [-1, 0], [-1, 0]],
    [[9, 0.5], [18, 1], [-1, 0], [-1, 0]],
    [[-1, 0], [-1, 0], [10, 1], [-1, 0]],
    [[24, 1], [28, 1], [25, 1.5], [10, 1]],
    [[10, 1.5], [25, 1], [22, 1.5], [12, 1]],
    [[7, 1.5], [20, 1], [19, 1], [14, 1]],
    [[18, 1], [-1, 0], [-1, 0], [-1, 0]],
    [[12, 1.5], [22, 1], [21, 1], [18, 1]],
    [[20, 1], [-1, 0], [-1, 0], [-1, 0]],
    [[17, 1.5], [26, 1], [23, 1], [20, 1]],
    [[22, 1], [-1, 0], [-1, 0], [-1, 0]],
    [[-1, 0], [-1, 0], [16, 1], [-1, 0]],
    [[16, 1.5], [29, 1], [26, 1.5], [17, 1]],
    [[25, 1.5], [34, 1], [27, 1], [22, 1]],
    [[26, 1], [-1, 0], [-1, 0], [-1, 0]],
    [[-1, 0], [-1, 0], [35, 0.5], [16, 1]],
    [[35, 1], [32, 1], [30, 1], [25, 1]],
    [[29, 1], [33, 1], [34, 0.5], [-1, 0]],
    [[-1, 0], [-1, 0], [-1, 0], [35, 1]],
    [[-1, 0], [-1, 0], [-1, 0], [29, 1]],
    [[-1, 0], [-1, 0], [-1, 0], [30, 1]],
    [[30, 0.5], [-1, 0], [-1, 0], [26, 1]],
    [[28, 0.5], [31, 1], [29, 1], [-1, 0]],
]

# Define the current and target points
current_point = 19
target_point = 33

# Find the fastest routes
fastest_routes = find_fastest_routes(points, current_point, target_point)
print(fastest_routes)


def getPathWithLessTurns(routes, points):
    min_turns = float("inf")
    best_route = None
    for route in routes:
        turns = 0
        for i in range(len(route) - 2):
            current_point = route[i]
            next_point = route[i + 1]
            next_next_point = route[i + 2]
            for x in points[current_point]:
                if x[0] == next_point:
                    current_index = points[current_point].index(x)
                    break

            for x in points[next_point]:
                if x[0] == next_next_point:
                    next_index = points[next_point].index(x)
                    break

            if abs(current_index - next_index) != 0:
                turns += 1

        if turns < min_turns:
            min_turns = turns
            best_route = route

    return best_route


print(getPathWithLessTurns(fastest_routes, points))


def getDistanceToPointFromPoint(points, current_point, target_point):
    for i in range(len(points[current_point])):
        if points[current_point][i][0] == target_point:
            return points[current_point][i][1]


print("distance from points", getDistanceToPointFromPoint(points, 26, 27))


def newPath(points, route):
    newPath = []
    distance = getDistanceToPointFromPoint(points, route[0], route[1])

    for i in range(len(route) - 2):
        current_point = route[i]
        next_point = route[i + 1]
        next_next_point = route[i + 2]
        print("current point", current_point, next_point, next_next_point)
        for x in points[current_point]:
            if x[0] == next_point:
                current_index = points[current_point].index(x)
                break

        for x in points[next_point]:
            if x[0] == next_next_point:
                next_index = points[next_point].index(x)
                break

        if abs(current_index - next_index) != 0:
            newPath.append([next_point, distance])
            distance = getDistanceToPointFromPoint(points, next_point, next_next_point)
        else:
            distance += points[current_point][current_index][1]

    newPath.append(
        [
            route[-1],
            distance,
        ]
    )
    return newPath


print(newPath(points, getPathWithLessTurns(fastest_routes, points)))


def getDirectionFromPointToPoint(route, newRoute):
    direction = []
    print(newRoute[0])
    print(newRoute[0])
    for x in range(len(newRoute)):
        if x == 0:
            direction.append(
                [
                    getDirectionFromOnePointToPoint(
                        route[x],
                        getNextPointFromPoint(route, route[x]),
                    ),
                    newRoute[x][1],
                ]
            )
        else:
            direction.append(
                [
                    getDirectionFromOnePointToPoint(
                        newRoute[x - 1][0],
                        getNextPointFromPoint(route, newRoute[x - 1][0]),
                    ),
                    newRoute[x][1],
                ]
            )

    return direction


def getNextPointFromPoint(route, point):
    for i in range(len(route) - 1):
        if route[i] == point:
            return route[i + 1]


print("getNe", getNextPointFromPoint(getPathWithLessTurns(fastest_routes, points), 34))


def getDirectionFromOnePointToPoint(pointOne, pointTwo):
    print(pointOne, pointTwo)
    for i in range(4):
        if points[pointOne][i][0] == pointTwo:
            print("directions", pointOne, pointTwo, i)
            return i


print("get dir", getDirectionFromOnePointToPoint(13, 9))

print(
    getDirectionFromPointToPoint(
        getPathWithLessTurns(fastest_routes, points),
        newPath(points, getPathWithLessTurns(fastest_routes, points)),
    )
)


def rotate(current, wanted):
    angle = (wanted - current) * 90
    if angle > 180:
        angle = 360 - angle
    elif angle < -180:
        angle = 360 + angle
    return angle


while True:
    current_time = time.time()
    current_time_from_start = current_time - START_TIME

    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = deviation * abs(deviation) / TURN_RATE_DIVIDER * turn_rate_multiplier

    robot.drive(DRIVE_SPEED, turn_rate)

    grabber_motor.run(GRAB_SPEED)

    if current_time_from_start > 10 and not wentToMiddle:
        needToGoMiddle()

    if stop_watch.time > TIME_TO_STOP and gettingCover:
        ev3.speaker.beep()
        break
