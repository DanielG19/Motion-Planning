from enum import Enum
from queue import PriorityQueue
import numpy as np
import math
from bresenham import bresenham

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    # Actions are tuples corresponding to movements in (i, j)
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    LEFTUP = (-1, -1, 2 ** (1/2))
    RIGHTUP = (-1, 1, 2 ** (1/2))
    LEFTDOWN = (1, -1, 2 ** (1/2))
    RIGHTDOWN = (1, 1, 2 ** (1/2))

    # Assign a new property that returns the cost of an action
    @property
    def cost(self):
        return self.value[2]
    # Assign a property that returns the action itself
    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    # First define a list of all possible actions
    valid = list(Action)
    # Retrieve the grid shape and position of the current node
    n, m = grid.shape[0] - 1, grid.shape[1] - 1 # n = 4, m = 5, if x or y when moving gets autside of the limits, then:
    x, y = current_node

    # check if the node is off the grid or it's an obstacle
    # If it is either, remove the action that takes you there
    #UP
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    #DOWN
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    #LEFT
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    #RIGHT
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)
    #RIGHTUP
    if y + 1 > m or x - 1 < 0 or grid[x - 1, y + 1] == 1:
        valid.remove(Action.RIGHTUP)
    #RIGHTDOWN
    if y + 1 > n or x + 1 > n or grid[x + 1, y + 1] == 1:
        valid.remove(Action.RIGHTDOWN)
    #LEFTUP
    if y - 1 < 0 or x - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid.remove(Action.LEFTUP)
    #LEFTDOWN
    if y - 1 < 0 or x + 1 > n or grid[x + 1, y - 1] == 1:
        valid.remove(Action.LEFTDOWN)

    return valid


def a_star(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def find_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]

    return near_start, near_goal

def BresenhamFun(path,grid,numCells):
    # bresenham every 4 cells
    # bresenham create a line from p1 to p2, p2 will be decided by the numCells i want to iterate.
    # numCells: How many cells i'll check per iteration.
    i = 0
    j = numCells - 1
    cells = path.copy()
    for _ in range(math.floor(len(path) / numCells)):
        if (j < len(path)):
            p1 = path[i]
            p2 = path[j]
            line = (p1[0], p1[1], p2[0], p2[1])
            Tempcells = list(bresenham(line[0], line[1], line[2], line[3]))
            CollitionFlag = 0
            for cell in Tempcells:
                if grid[cell[0], cell[1]] == 1:
                    CollitionFlag = 1
                    i = i + numCells - 1
                    j = j + numCells - 1
                    break
            if CollitionFlag == 0:
                del cells[i + 1:j]  # "del" means Delete
                i= i + 1
                j = j + 1


    return cells;
