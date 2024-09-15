import numpy as np
from math import pi

steps = 500

start = np.array([0.1, -0.2, 0.42600, 0.0, 0, pi / 4])
end = np.array([-0.2, -0.5, 0.426, 0.0, 0, pi / 4])

diff = end - start

trajectory = []
for step in range(steps):
    trajectory.append(start + (step / steps) * diff)

np.savetxt(
    "path/to/output.csv",
    np.array(trajectory),
    delimiter=",",
    fmt="%.5f",
)
