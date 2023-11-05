# dodelat 6, 7, 9
# array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [1, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]


# def find_best_route(points, current_point, target_point):
#     def dfs(current, path, time):
#         if current == target_point:
#             return path
#         if current < 0 or current >= len(points):
#             return None

#         best_path = None
#         best_time = float("inf")

#         for i, (next_point, travel_time) in enumerate(points[current]):
#             if next_point not in path:
#                 new_path = path + [next_point]
#                 new_time = time + travel_time

#                 if new_time < best_time:
#                     result = dfs(next_point, new_path, new_time)
#                     if result is not None:
#                         best_path = result
#                         best_time = new_time

#         return best_path

#     initial_path = [current_point]
#     return dfs(current_point, initial_path, 0)


# # Define your array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [1, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]

# # Define the current and target points
# current_point = 0
# target_point = 8

# # Find the best route
# best_route = find_best_route(points, current_point, target_point)
# print(best_route)


# funguje
# def find_fastest_route(points, current_point, target_point):
#     def dfs(current, path, time):
#         if current == target_point:
#             return path, time
#         if current < 0 or current >= len(points):
#             return None, float("inf")

#         best_path = None
#         best_time = float("inf")

#         for i, (next_point, travel_time) in enumerate(points[current]):
#             if next_point not in path:
#                 new_path = path + [next_point]
#                 new_time = time + travel_time

#                 result_path, result_time = dfs(next_point, new_path, new_time)

#                 if result_time < best_time:
#                     best_path = result_path
#                     best_time = result_time

#         return best_path, best_time

#     initial_path = [current_point]
#     path, time = dfs(current_point, initial_path, 0)
#     return path


# # Define your array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [1, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]

# # Define the current and target points
# current_point = 6
# target_point = 9

# # Find the fastest route
# fastest_route = find_fastest_route(points, current_point, target_point)
# print(fastest_route)


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
    [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
    [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
    [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
    [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
]

# Define the current and target points
current_point = 6
target_point = 9

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
    distance = 0
    for i in range(len(points[current_point])):
        if points[current_point][i][0] == target_point:
            distance = points[current_point][i][1]
            break
    return distance


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
            distance = 0
        else:
            distance += points[current_point][current_index][1]

    newPath.append(
        [
            route[-1],
            distance + getDistanceToPointFromPoint(points, route[-2], route[-1]),
        ]
    )
    return newPath


print(newPath(points, getPathWithLessTurns(fastest_routes, points)))


def getDirectionFromPointToPoint(route, newRoute):
    


print(getDirectionFromPointToPoint(getPathWithLessTurns(fastest_routes, points),    ))

# def find_fastest_route_with_min_turns(points, current_point, target_point):
#     def dfs(current, path, time, turns):
#         if current < 0 or current >= len(points):
#             return None, float("inf"), float("inf")

#         best_path = None
#         best_time = float("inf")
#         best_turns = float("inf")

#         for i, (next_point, travel_time) in enumerate(points[current]):
#             if next_point not in path:
#                 new_path = path + [next_point]
#                 new_time = time + travel_time
#                 new_turns = turns

#                 if len(path) >= 2:
#                     prev_point = path[-2]
#                     prev_index = next(
#                         (
#                             i
#                             for i, (p, _) in enumerate(points[current])
#                             if p == prev_point
#                         ),
#                         None,
#                     )
#                     next_index = next(
#                         (
#                             i
#                             for i, (p, _) in enumerate(points[current])
#                             if p == next_point
#                         ),
#                         None,
#                     )

#                     if prev_index is not None and next_index is not None:
#                         if abs(prev_index - next_index) != 0:
#                             new_turns += 1
#                             print(
#                                 path,
#                                 "Turns:",
#                                 new_turns,
#                                 path[-2],
#                                 "->",
#                                 path[-1],
#                                 prev_index,
#                                 next_index,
#                             )

#                 result_path, result_time, result_turns = dfs(
#                     next_point, new_path, new_time, new_turns
#                 )
#                 if result_time < best_time or (
#                     result_time == best_time and result_turns < best_turns
#                 ):
#                     best_path = result_path
#                     best_time = result_time
#                     best_turns = result_turns

#         if current == target_point and best_path == path:
#             return path, time, turns

#         return best_path, best_time, best_turns

#     initial_path = [current_point]
#     path, time, turns = dfs(current_point, initial_path, 0, 0)
#     return path, turns


# # Define your array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [1, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]

# # Define the current and target points
# current_point = 6
# target_point = 9

# # Find the fastest route with the fewest turns
# fastest_route = find_fastest_route_with_min_turns(points, current_point, target_point)
# print(fastest_route)
#
#
#
#
#
#
#


# def find_fastest_route_with_min_turns(points, current_point, target_point):
#     def dfs(current, path, time, turns):
#         if current == target_point:
#             return path, time, turns
#         if current < 0 or current >= len(points):
#             return None, float("inf"), float("inf")

#         best_path = None
#         best_time = float("inf")
#         best_turns = float("inf")

#         for i, (next_point, travel_time) in enumerate(points[current]):
#             if next_point not in path:
#                 new_path = path + [next_point]
#                 new_time = time + travel_time
#                 new_turns = turns

#                 if len(path) > 1:
#                     prev_point = path[-1]
#                     prev_index = next(
#                         (
#                             i
#                             for i, (p, _) in enumerate(points[current])
#                             if p == prev_point
#                         ),
#                         None,
#                     )
#                     next_index = next(
#                         (
#                             i
#                             for i, (p, _) in enumerate(points[current])
#                             if p == next_point
#                         ),
#                         None,
#                     )

#                     if prev_index is not None and next_index is not None:
#                         if abs(prev_index - next_index) == 2:
#                             new_turns += 1

#                 result_path, result_time, result_turns = dfs(
#                     next_point, new_path, new_time, new_turns
#                 )

#                 if result_time < best_time or (
#                     result_time == best_time and result_turns < best_turns
#                 ):
#                     best_path = result_path
#                     best_time = result_time
#                     best_turns = result_turns

#         return best_path, best_time, best_turns

#     initial_path = [current_point]
#     path, time, turns = dfs(current_point, initial_path, 0, 0)
#     return path


# # Define your array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [5, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]

# # Define the current and target points
# current_point = 6
# target_point = 9

# # Find the fastest route with the fewest turns
# fastest_route = find_fastest_route_with_min_turns(points, current_point, target_point)
# print(fastest_route)


#
#
#
#
#
#


# def calculate_path_length(points, path):
#     length = 0

#     for i in range(len(path) - 1):
#         current_point = path[i]
#         next_point = path[i + 1]

#         # Find the travel time from the current point to the next point
#         for _, travel_time in points[current_point]:
#             if _ == next_point:
#                 length += travel_time
#                 print("Travel time:", travel_time, path[i], "->", path[i + 1])
#                 break

#     return length


# # Define your array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [1, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]

# # Define the path
# path = [6, 4, 1, 0, 2, 9]

# # Calculate the length of the path
# path_length = calculate_path_length(points, path)
# print("Length of the path:", path_length)


# def find_route_with_fewest_turns(points, current_point, target_point):
#     def dfs(current, path, time, turns, turn_positions):
#         if current == target_point:
#             return path, time, turns, turn_positions
#         if current < 0 or current >= len(points):
#             return None, float("inf"), float("inf"), []

#         best_path = None
#         best_time = float("inf")
#         best_turns = float("inf")
#         best_turn_positions = []

#         for i, (next_point, travel_time) in enumerate(points[current]):
#             if next_point not in path:
#                 new_path = path + [next_point]
#                 new_time = time + travel_time

#                 # Count turns if the previous point is valid
#                 if len(path) >= 2 and path[-2] != -1:
#                     prev_point = path[-2]
#                     turns_with_new_point = (
#                         turns + 1 if next_point != points[prev_point][i][0] else turns
#                     )
#                 else:
#                     turns_with_new_point = turns

#                 new_turn_positions = turn_positions + [i] if i != -1 else turn_positions

#                 result_path, result_time, result_turns, result_turn_positions = dfs(
#                     next_point,
#                     new_path,
#                     new_time,
#                     turns_with_new_point,
#                     new_turn_positions,
#                 )

#                 if result_time < best_time or (
#                     result_time == best_time and result_turns < best_turns
#                 ):
#                     best_path = result_path
#                     best_time = result_time
#                     best_turns = result_turns
#                     best_turn_positions = result_turn_positions

#         return best_path, best_time, best_turns, best_turn_positions

#     initial_path = [current_point]
#     path, time, turns, turn_positions = dfs(current_point, initial_path, 0, 0, [])
#     return path, turns, turn_positions


# # Define your array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [1, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]

# # Define the current and target points
# current_point = 6
# target_point = 9

# # Find the route with fewest turns and their positions
# best_route, num_turns, turn_positions = find_route_with_fewest_turns(
#     points, current_point, target_point
# )
# print("Best Route:", best_route)
# print("Number of Turns:", num_turns)
# print("Turn Positions:", turn_positions)


# def find_route_with_fewest_turns(points, current_point, target_point):
#     def dfs(current, path, time, turns):
#         if current == target_point:
#             return path, time, turns
#         if current < 0 or current >= len(points):
#             return None, float("inf"), float("inf")

#         best_path = None
#         best_time = float("inf")
#         best_turns = float("inf")

#         for i, (next_point, travel_time) in enumerate(points[current]):
#             if next_point not in path:
#                 new_path = path + [next_point]
#                 new_time = time + travel_time

#                 # Count turns if the previous point is valid
#                 if len(path) >= 2 and path[-2] != -1:
#                     prev_point = path[-2]
#                     turns_with_new_point = (
#                         turns + 1 if next_point != points[prev_point][i][0] else turns
#                     )
#                 else:
#                     turns_with_new_point = turns

#                 result_path, result_time, result_turns = dfs(
#                     next_point, new_path, new_time, turns_with_new_point
#                 )

#                 if result_time < best_time or (
#                     result_time == best_time and result_turns < best_turns
#                 ):
#                     best_path = result_path
#                     best_time = result_time
#                     best_turns = result_turns

#         return best_path, best_time, best_turns

#     initial_path = [current_point]
#     path, time, turns = dfs(current_point, initial_path, 0, 0)

#     # Check for a turn right before the end

#     if len(path) >= 2 and path[-2] != -1 and path[-1] != target_point:
#         prev_point = path[-2]
#         if path[-1] != points[prev_point][i][0]:
#             turns += 1

#     return path


# # Define your array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [1, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]

# # Define the current and target points
# current_point = 6
# target_point = 9

# # Find the route with fewest turns
# best_route = find_route_with_fewest_turns(points, current_point, target_point)
# print(best_route)


# def find_route_with_fewest_turns(points, current_point, target_point):
#     def is_turn(path):
#         if len(path) < 3:
#             return False
#         prev_point = path[-3]
#         current_point = path[-2]
#         next_point = path[-1]
#         return prev_point != -1 and current_point != -1 and next_point != -1

#     def dfs(current, path, time, turns):
#         if current == target_point:
#             return path, time, turns
#         if current < 0 or current >= len(points):
#             return None, float("inf"), float("inf")

#         best_path = None
#         best_time = float("inf")
#         best_turns = float("inf")

#         for i, (next_point, travel_time) in enumerate(points[current]):
#             if next_point not in path:
#                 new_path = path + [next_point]
#                 new_time = time + travel_time
#                 turns_with_new_point = turns + int(is_turn(new_path))

#                 result_path, result_time, result_turns = dfs(
#                     next_point, new_path, new_time, turns_with_new_point
#                 )

#                 if result_time < best_time or (
#                     result_time == best_time and result_turns < best_turns
#                 ):
#                     best_path = result_path
#                     best_time = result_time
#                     best_turns = result_turns

#         return best_path, best_time, best_turns

#     initial_path = [current_point]
#     path, time, turns = dfs(current_point, initial_path, 0, 0)

#     # Check for a turn right before the end
#     if is_turn(path) and path[-1] != target_point:
#         turns += 1

#     return path


# # Define your array of points
# points = [
#     [[1, 0.5], [-1, 0], [2, 1], [3, 1]],
#     [[-1, 0], [4, 1], [0, 0.5], [-1, 0]],
#     [[0, 1], [7, 1], [9, 1], [8, 1]],
#     [[-1, 0], [0, 1], [-1, 0], [-1, 0]],
#     [[5, 1], [6, 1], [7, 1.5], [1, 1]],
#     [[-1, 0], [-1, 0], [4, 1], [-1, 0]],
#     [[-1, 0], [-1, 0], [-1, 0], [4, 1]],
#     [[4, 1.5], [-1, 0], [-1, 0], [2, 1]],
#     [[-1, 0], [2, 1], [-1, 0], [-1, 0]],
#     [[2, 1], [-1, 0], [-1, 0], [-1, 0]],
# ]

# # Define the current and target points
# current_point = 6
# target_point = 9

# # Find the route with fewest turns
# best_route = find_route_with_fewest_turns(points, current_point, target_point)
# print(best_route)
