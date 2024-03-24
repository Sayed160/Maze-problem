import heapq
import math

# Define the maze
maze = [
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 1, 1, 1, 1, 0, 0],
    [1, 0, 1, 1, 0, 0, 1, 0, 1, 1],
    [0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1],
    [1, 1, 1, 0, 1, 1, 1, 1, 1, 1],
    [0, 1, 1, 0, 1, 1, 1, 1, 0, 0],
    [1, 1, 0, 0, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 0, 1, 1, 1, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]

# Define the dimensions of the maze
maze_height = len(maze)
maze_width = len(maze[0])

# Define possible movements (up, down, left, right, diagonal)
movements = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]

def heuristic_manhattan(start, goal):
    # Manhattan distance heuristic
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

def heuristic_euclidean(start, goal):
    # Euclidean distance heuristic
    return math.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)

def heuristic_diagonal(start, goal):
    # Diagonal distance heuristic
    dx = abs(start[0] - goal[0])
    dy = abs(start[1] - goal[1])
    return max(dx, dy)

def astar(start, goal, heuristic_func):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        current_cost, current_node = heapq.heappop(open_set)

        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            path.reverse()
            return path

        for move in movements:
            new_node = (current_node[0] + move[0], current_node[1] + move[1])
            if 0 <= new_node[0] < maze_height and 0 <= new_node[1] < maze_width and maze[new_node[0]][new_node[1]] == 1:
                new_cost = cost_so_far[current_node] + 1
                if new_node not in cost_so_far or new_cost < cost_so_far[new_node]:
                    cost_so_far[new_node] = new_cost
                    priority = new_cost + heuristic_func(goal, new_node)
                    heapq.heappush(open_set, (priority, new_node))
                    came_from[new_node] = current_node

# Define start and goal points
start_point = (0, 0)
goal_point = (9, 9)

# Find the optimal path using different heuristics
optimal_path_manhattan = astar(start_point, goal_point, heuristic_manhattan)
optimal_path_euclidean = astar(start_point, goal_point, heuristic_euclidean)
optimal_path_diagonal = astar(start_point, goal_point, heuristic_diagonal)

# Visualize the optimal paths
print("Optimal Path using Manhattan Distance:")
for i in range(maze_height):
    for j in range(maze_width):
        if (i, j) == start_point:
            print("S", end=" ")
        elif (i, j) == goal_point:
            print("G", end=" ")
        elif (i, j) in optimal_path_manhattan:
            print(".", end=" ")
        elif maze[i][j] == 0:
            print("#", end=" ")
        else:
            print(" ", end=" ")
    print()

print("\nOptimal Path using Euclidean Distance:")
for i in range(maze_height):
    for j in range(maze_width):
        if (i, j) == start_point:
            print("S", end=" ")
        elif (i, j) == goal_point:
            print("G", end=" ")
        elif (i, j) in optimal_path_euclidean:
            print(".", end=" ")
        elif maze[i][j] == 0:
            print("#", end=" ")
        else:
            print(" ", end=" ")
    print()

print("\nOptimal Path using Diagonal Distance:")
for i in range(maze_height):
    for j in range(maze_width):
        if (i, j) == start_point:
            print("S", end=" ")
        elif (i, j) == goal_point:
            print("G", end=" ")
        elif (i, j) in optimal_path_diagonal:
            print(".", end=" ")
        elif maze[i][j] == 0:
            print("#", end=" ")
        else:
            print(" ", end=" ")
    print()
