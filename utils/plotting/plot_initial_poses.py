import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from utils.utils import euler_to_matrix


def euler_to_vector(theta_x, theta_y, theta_z):
    """Convert Euler angles to a direction vector."""
    R = euler_to_matrix(
        theta_x,
        theta_y,
        theta_z,
    )

    # the choice of the first axis and the -1 multiplication is in order to transform to the "visual" dynamic sub-assembly direction, which is its negative x-axis as can be seen in the simulation.
    direction_vector = R @ np.array([1, 0, 0])
    return direction_vector * -1


def plot_vectors(csv_file):
    df = pd.read_csv(
        csv_file, header=None, names=["x", "y", "z", "theta_x", "theta_y", "theta_z"]
    )

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for index, row in df.iterrows():
        length = 0.05
        x, y, z = row["x"], row["y"], row["z"]
        theta_x, theta_y, theta_z = (row["theta_x"]), (row["theta_y"]), (row["theta_z"])

        direction = euler_to_vector(theta_x, theta_y, theta_z)
        ax.quiver(
            x,
            y,
            z,
            direction[0],
            direction[1],
            direction[2],
            length=length,
            normalize=True,
        )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")  # type: ignore

    plt.show()


initial_poses_file = "path/to/initial_poses.csv"
plot_vectors(initial_poses_file)
