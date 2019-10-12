import numpy as np

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


def max_pool(map, K):
    # First, trim the map down such that it can be divided evenly into K by K square sections.
    # Try to keep the trimming as symmetric as possible: If we trim the bottom side, trim the top side next, etc.
    H, W = map.shape
    K = K

    H_excess = H % K
    W_excess = W % K

    start_x = H_excess / 2
    end_x = H - (H_excess / 2)

    start_y = W_excess / 2
    end_y = W - (W_excess / 2)

    # In the event that we only need to trim one edge to make that dimension divisible by K, we have over-adjusted
    # in the above code. Rectify that here - is there a simple way to not make that mistake prior?
    if (H_excess % 2 == 1):
        end_x -= 1

    if (W_excess % 2 == 1):
        end_y -= 1

    map = map[start_x:end_x, start_y:end_y]  # Adjusted map that can now be divided into KxK sections

    # Divide the adjusted map into KxK sections, taking the max value of each section to be the value of that
    # section.
    # We can also take a running total of the number of 1's in each section, to determine which
    # sections are least likely to be impassable.
    HK = H // K
    WK = W // K

    weighted_map = (map[:HK * K, :WK * K].reshape(HK, K, WK, K).sum(axis=(1, 3)))

    print 'Weighted reduced map:'
    print weighted_map

    weighted_map[weighted_map > 0] *= -1
    weighted_map[weighted_map == 0] = 1

    grid = Grid(matrix=weighted_map)
    start = grid.node(2, 0)
    end = grid.node(0, 2)

    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)

    path_found = (len(path) != 0)
    threshold = 0
    while not path_found:
        threshold -= 1
        weighted_map[weighted_map == threshold] = 1
        grid = Grid(matrix=weighted_map)
        start = grid.node(2, 0)
        end = grid.node(0, 2)

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        path_found = (len(path) != 0)

    print(path)
    print('operations:', runs, 'path length:', len(path))
    print(grid.grid_str(path=path, start=start, end=end))
    print 'Highest weight allowed to drive over: ', threshold * -1

    adj_path = np.array(path)
    adj_path = K * adj_path + (K/2)
    print adj_path

    for pt in adj_path[:-1]:
        # computeEasyPath(pt, pt + 1, stepSize?????)
        print('hey') #placeholder so the red squiggly leaves me alone

    return weighted_map


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

    K = 3

    max_pool(map, K)



