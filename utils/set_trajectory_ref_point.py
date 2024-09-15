import numpy as np
from utils.utils import euler_to_matrix


def set_trajectory_ref_point(
    trajectory, ref_point_offset, dynamic_sub_assembly_relative_rotation_mat
):
    for point_idx in range(len(trajectory)):
        point = trajectory[point_idx]
        point_rot_mat = euler_to_matrix(point[3], point[4], point[5])
        ref_point = point[:3] + np.dot(
            np.dot(point_rot_mat, dynamic_sub_assembly_relative_rotation_mat),
            ref_point_offset,
        )
        trajectory[point_idx][:3] = ref_point
    return trajectory
