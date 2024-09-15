import numpy as np
from math import degrees

np.set_printoptions(suppress=True, precision=6)
trajectory_file_path = "path/to/input.csv"
trajectory = np.genfromtxt(
    trajectory_file_path,
    delimiter=",",
    skip_header=1,
)[:-1]
diffs = []
for idx in range(len(trajectory) - 1):
    diffs.append(degrees(np.max(np.abs(trajectory[idx] - trajectory[idx + 1]))))
print(np.argmax(diffs))
print(np.max(diffs))
