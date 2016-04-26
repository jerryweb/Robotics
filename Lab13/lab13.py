
import numpy as np
import math
import matplotlib.pyplot as plt

def adjacent(m, pos):
    """
    Computes a list of adjacent cells for the given map and position
    :param m: Map of the environment (numpy matrix)
    :param pos: Query Position Tuple (row, column), 0-based
    :return: list of adjacent positions
    """
    result = []
    if pos[0] > 0 and m[pos[0] - 1, pos[1]] == 0:
        result.append((pos[0] - 1, pos[1]))
    if pos[0] < m.shape[0] - 1 and m[pos[0] + 1, pos[1]] == 0:
        result.append((pos[0] + 1, pos[1]))
    if pos[1] > 0 and m[pos[0], pos[1] - 1] == 0:
        result.append((pos[0], pos[1] - 1))
    if pos[1] < m.shape[1] - 1 and m[pos[0], pos[1] + 1] == 0:
        result.append((pos[0], pos[1] + 1))
    return result

class Node:
    def __init__(self):
        self.f_score = 0            # total score
        self.g_score = 0            # cost from start node
        self.heuristic_score = 0
        self.parent_node = None
        self.child_node = [0,1]
        self.position = [0,0]
        self.is_start_node = False




def a_star(m, start_pos, goal_pos):
    # Add code here
    closedSet = []
    openSet = []


    # create start node
    start_node = Node()
    start_node.position = start_pos
    start_node.heuristic_score = calculate_manhattan_distance(start_pos, goal_pos)
    start_node.f_score = start_node.heuristic_score + start_node.g_score
    start_node.is_start_node = True
    openSet.append(start_node)




    while len(openSet) != 0:
        min_value_node = Node()
        min_value_node.f_score = math.inf

        for i in range(0, len(openSet),1):
            # min_value.append(openSet[i].f_score)
            if openSet[i].f_score < min_value_node.f_score:
                min_value_node = openSet[i]


        if min_value_node.position == goal_pos:
            print("found path")
            return path(min_value_node.parent, min_value_node)

        #Remove N from open_set and add to closed_set
        if min_value_node in openSet:
            openSet.remove(min_value_node)
            closedSet.append(min_value_node)

        adjacent_cells = adjacent(m, min_value_node.position)

        #  check if node A is in the closedSet
        # for each neighbor node adjacent to node N
        for k in range(0, len(adjacent_cells), 1):
            A = generate_node(adjacent_cells[k], goal_pos)

            for l in range(0, len(closedSet), 1):
                if closedSet[l] == A:
                    continue
            if A in closedSet:
                continue
            else:
                temp_g_score = min_value_node.g_score + movement_cost(min_value_node.position, A.position)

                if A not in openSet or temp_g_score < A.g_score:
                    A.parent = min_value_node
                    A.g_score = temp_g_score
                    A.f_score = A.g_score + calculate_manhattan_distance(A.position, goal_pos)

                    if A not in openSet:
                        openSet.append(A)

    # return failure
    return start_pos


def path(parent, current):

    reconstructed_path = []

    while not current.is_start_node:
        reconstructed_path.append(current.position)
        print(current.position)
        current = current.parent

    reconstructed_path.append(current.position)
    return reconstructed_path


def generate_node(position, goal_pos):
    node = Node()
    node.position = position
    node.heuristic_score = calculate_manhattan_distance(position, goal_pos)
    node.f_score = node.heuristic_score + node.g_score
    return node

def movement_cost(current, neighbor):
    distance = math.fabs(current[0] - neighbor[0]) + math.fabs(current[1] - neighbor[1])
    return distance

def calculate_manhattan_distance(start, goal):
    distance = math.fabs(goal[0] - start[0]) + math.fabs(goal[1] - start[1])
    # print("manhattan distance for ", start," = ", distance)
    return distance



if __name__ == "__main__":
    # load the map
    m = np.loadtxt("map2.txt", delimiter=",")
    path = a_star(m, (9, 13), (5, 13))
    # print(len(path))
    # Add the path to the map (for visualization)
    # for i in range(0, len(path),1):
    #     print(path[i])
    for p in path:
        m[p] = 128
    # change the values (for better visualization)
    m[m == 0] = 255
    m[m == 1] = 0
    # Plot the map (& result)
    plt.matshow(m, cmap=plt.cm.gray)
    plt.show()
