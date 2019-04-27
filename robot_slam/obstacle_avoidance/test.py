
import numpy as np


full_map = np.ones(shape=(3,3))

local_map = np.array([[0, 0, 0],
                      [0, 5, 0],
                      [0, 0, 0]])

np.copyto(full_map, local_map, where=(local_map != 0))

print(full_map)

