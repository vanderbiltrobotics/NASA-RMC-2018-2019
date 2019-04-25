import numpy as np

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

if __name__ == "__main__":
    map = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 0, 0, 0, 1],
                    [1, 0, 1, 1, 0, 0, 0, 0, 1],
                    [1, 1, 1, 1, 0, 0, 0, 0, 0],
                    [1, 0, 0, 1, 0, 0, 0, 0, 0],
                    [1, 0, 0, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 0, 1],
                    [0, 0, 0, 0, 0, 1, 0, 0, 1],
                    [0, 0, 0, 1, 0, 0, 0, 0, 1]])

    # we want a buffer of maybe 20 cm on each edge; then, cut map down until it's divisible by whatever
    # dimension our zoom out will be.
    M, N = map.shape
    K = 3
    L = 3

    # This is probably inefficient... But the idea is, we start with the map having a buffer of size one
    # on each edge. Then, if this isn't properly divisible, we keep trimming the edges in a specific order until
    # the dimensions are indeed divisible by the dimensions of the sections we want to put together.
    i1 = 1
    i2 = 1
    while (M - i1 - i2) % K != 0:
        if i1 <= i2:
            i1 += 1
        else:
            i2 += 1

    j1 = 1
    j2 = 1
    while (N - j1 - j2) % L != 0:
        if j1 <= j2:
            j1 += 1
        else:
            j2 += 1

    new_map = np.zeros((M - i1 - i2, N - j1 - j2,))
    new_map = np.maximum(new_map, map[i1: M - i1 - i2 + 2, j1: N - j1 - j2 + 2])

    MK = (M - i1 - i2) / K
    NL = (N - j1 - j2) / L

    small_map = (new_map[:MK*K, :NL*L].reshape(MK, K, NL, L).max(axis=(1, 3)))


    print(small_map)
