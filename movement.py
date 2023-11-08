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
current_point = 13
target_point = 33
current_point = 13
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


def newPath(points, route):
    newPath = []
    distance = getDistanceToPointFromPoint(points, route[0], route[1])

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
        # for i in range(len(points[route[x]])):
        #     if points[route[x]][i][0] == route[x + 1]:
        #         direction.append([i, newRoute[x][1]])

    return direction


# je to jenom posunuty o jedno doleva


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
