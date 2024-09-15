import numpy as np

trajectory_file_path = "path/to/input.csv"
trajectory = np.genfromtxt(
    trajectory_file_path,
    delimiter=",",
    skip_header=1,
)[:-1]

averages = (trajectory[:-1] + trajectory[1:]) / 2

extended_trajectory = np.empty((trajectory.shape[0] * 2 - 1, trajectory.shape[1]))
extended_trajectory[0::2] = trajectory
extended_trajectory[1::2] = averages

np.savetxt(
    "path/to/output.csv",
    np.array(extended_trajectory),
    delimiter=",",
    fmt="%.10f",
)
