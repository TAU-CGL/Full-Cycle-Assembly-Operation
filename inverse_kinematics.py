import json
import os
import numpy as np
from math import cos, sin, atan2, sqrt, pi
from typing import Tuple, List
from numpy.typing import NDArray
import networkx as nx

from collision_detection import check_collision
from utils.set_trajectory_ref_point import set_trajectory_ref_point
from utils.utils import (
    euler_to_matrix,
    get_euler_angles_from_matrix,
    get_transformation_matrix,
    min_max_edge_dijkstra,
)

np.set_printoptions(suppress=True, precision=6)


def get_ijk_params_from_branch(
    shoulder_left: float, wrist_rh: float, elbow_up: float
) -> Tuple[float, float, float]:
    i = (shoulder_left - 0.5) * 2
    j = (wrist_rh - 0.5) * 2
    k = i * ((elbow_up - 0.5) * 2)
    return i, j, k


def get_inverse_kinematics(
    dh: Tuple[float, ...],
    transform: NDArray[np.float64],
    shoulder_left: float,
    wrist_rh: float,
    elbow_up: float,
    theta_offsets: NDArray[np.float64],
) -> Tuple[float, float, float, float, float, float] | None:
    a_2, a_3, d_1, d_4, d_5, d_6 = dh
    i, j, k = get_ijk_params_from_branch(shoulder_left, wrist_rh, elbow_up)
    r = transform[:3, :3]
    p = transform[:3, 3]
    A = p[1] - d_6 * r[1, 2]
    B = p[0] - d_6 * r[0, 2]
    if B**2 + A**2 - d_4**2 < 0:
        return None
    theta_1 = i * atan2(sqrt(B**2 + A**2 - d_4**2), d_4) + atan2(B, -A)
    s_1 = sin(theta_1)
    c_1 = cos(theta_1)
    C = c_1 * r[0, 0] + s_1 * r[1, 0]
    D = c_1 * r[1, 1] - s_1 * r[0, 1]
    E = s_1 * r[0, 0] - c_1 * r[1, 0]
    theta_5 = j * atan2(sqrt(E**2 + D**2), s_1 * r[0, 2] - c_1 * r[1, 2])
    s_5 = sin(theta_5)
    c_5 = cos(theta_5)
    theta_6 = atan2(D / s_5, E / s_5)
    s_6 = sin(theta_6)
    c_6 = cos(theta_6)
    F = c_5 * c_6
    theta_234 = atan2(r[2, 0] * F - s_6 * C, F * C + s_6 * r[2, 0])
    s_234 = sin(theta_234)
    c_234 = cos(theta_234)
    K_C = c_1 * p[0] + s_1 * p[1] - s_234 * d_5 + c_234 * s_5 * d_6
    K_S = p[2] - d_1 + c_234 * d_5 + s_234 * s_5 * d_6
    c_3 = (K_S**2 + K_C**2 - a_2**2 - a_3**2) / (2 * a_2 * a_3)
    if c_3**2 > 1:
        return None
    s_3 = sqrt(1 - c_3**2)
    theta_3 = k * atan2(s_3, c_3)
    s_3 = sin(theta_3)
    c_3 = cos(theta_3)
    theta_2 = atan2(K_S, K_C) - atan2(s_3 * a_3, c_3 * a_3 + a_2)
    theta_4 = theta_234 - theta_2 - theta_3
    return (
        theta_1 + theta_offsets[0],
        theta_2 + theta_offsets[1],
        theta_3 + theta_offsets[2],
        theta_4 + theta_offsets[3],
        theta_5 + theta_offsets[4],
        (((theta_6 + theta_offsets[5]) / pi * 180) % 360) * pi / 180,
    )


def adjust_trajectory(ik_trajectory):
    for j in range(ik_trajectory.shape[1]):
        while abs(ik_trajectory[0][j]) > pi:
            if ik_trajectory[0][j] > pi:
                ik_trajectory[0, j] -= 2 * pi
            elif ik_trajectory[0][j] < -pi:
                ik_trajectory[0, j] += 2 * pi
    for i in range(1, ik_trajectory.shape[0]):
        for j in range(ik_trajectory.shape[1]):
            diff = 2 * pi
            while abs(diff) > pi:
                diff = ik_trajectory[i, j] - ik_trajectory[i - 1, j]
                if diff > pi:
                    ik_trajectory[i, j] -= 2 * pi
                elif diff < -pi:
                    ik_trajectory[i, j] += 2 * pi
    return ik_trajectory


def place_trajectory(
    trajectory,
    initial_pose,
    dynamic_sub_assembly_relative_rotation,
    position_factor,
    ref_point_offset,
):
    dynamic_sub_assembly_relative_rotation_mat = euler_to_matrix(
        dynamic_sub_assembly_relative_rotation[0],
        dynamic_sub_assembly_relative_rotation[1],
        dynamic_sub_assembly_relative_rotation[2],
    )
    init_rot_matrix = np.dot(
        euler_to_matrix(initial_pose[3], initial_pose[4], initial_pose[5]),
        np.linalg.inv(
            euler_to_matrix(
                trajectory[0][3], trajectory[0][4], trajectory[0][5]
            )  # inverse of initial dynamic sub assembly angle relative to static sub assembly
        ),
    )
    inv_init_rot_matrix = np.linalg.inv(init_rot_matrix)
    trajectory[:, :3] *= position_factor
    trajectory[:, :3] = np.dot(
        trajectory[:, :3] - trajectory[0, :3], inv_init_rot_matrix
    )
    offset = initial_pose[:3] - trajectory[0, :3]
    trajectory[:, :3] += offset
    offset_trajectory = trajectory
    for i in range(len(offset_trajectory)):
        original = offset_trajectory[i, 3:]
        mat = euler_to_matrix(original[0], original[1], original[2])
        rotated_mat = np.dot(
            np.dot(init_rot_matrix, mat),
            np.linalg.inv(dynamic_sub_assembly_relative_rotation_mat),
        )
        rotated_euler = np.array(get_euler_angles_from_matrix(rotated_mat))
        offset_trajectory[i, 3:] = rotated_euler
    offset_trajectory = set_trajectory_ref_point(
        np.copy(offset_trajectory),
        ref_point_offset,
        dynamic_sub_assembly_relative_rotation_mat,
    )
    return offset_trajectory


def generate_ik_of_trajectory(
    trajectory_file: str,
    dh: Tuple[float, ...],
    initial_pose: List[float],
    position_factor: float,
    dynamic_sub_assembly_relative_rotation: List[float],
    ref_point_offset: NDArray[np.float64],
    theta_offsets: NDArray[np.float64],
    robot_id: str,
    output_path: str,
    test_AB=False,
    delta=10,
) -> Tuple[int, float, bool]:
    trajectory = np.genfromtxt(
        trajectory_file,
        delimiter=",",
        skip_header=1,
    )
    placed_trajectory = place_trajectory(
        trajectory,
        initial_pose,
        dynamic_sub_assembly_relative_rotation,
        position_factor,
        ref_point_offset,
    )
    G = nx.DiGraph()
    G.add_node("s")
    G.add_node("t")
    previous_pose_solutions: list[NDArray] = []
    if test_AB:
        too_close_to_base = (
            np.min(np.linalg.norm(placed_trajectory[:, :2], axis=1)) < dh[3]
        )
        if too_close_to_base:
            return -1, 0, False
    for i in range(len(placed_trajectory)):
        T = get_transformation_matrix(
            placed_trajectory[i, 0],
            placed_trajectory[i, 1],
            placed_trajectory[i, 2],
            placed_trajectory[i, 3],
            placed_trajectory[i, 4],
            placed_trajectory[i, 5],
        )
        current_pose_solutions = []
        trajectory_blocked = True
        for shoulder in [0, 1]:
            for wrist in [0, 1]:
                for elbow in [1]:
                    # considering only "elbow up" branches, change elbow range to [0, 1] to consider elbow down as well
                    ik = get_inverse_kinematics(
                        dh, T, shoulder, wrist, elbow, theta_offsets
                    )
                    if ik != None:
                        solution_node = np.array(ik)
                        collision_detected = check_collision(solution_node, robot_id)
                        if not collision_detected:
                            node_added = False
                            if i == 0:
                                current_pose_solutions.append(solution_node)
                                G.add_node(np.array2string(solution_node))
                            for node in previous_pose_solutions:
                                dists = (np.abs(solution_node - node) * 180 / pi) % 360
                                dists = np.minimum(dists, 360 - dists)
                                weight = np.max(dists)
                                if weight < delta:
                                    if not node_added:
                                        current_pose_solutions.append(solution_node)
                                        G.add_node(np.array2string(solution_node))
                                        node_added = True
                                    trajectory_blocked = False
                                    G.add_edge(
                                        np.array2string(node),
                                        np.array2string(solution_node),
                                        weight=weight,
                                    )
        if i == 0:
            for node in current_pose_solutions:
                trajectory_blocked = False
                G.add_edge("s", np.array2string(node))
        if trajectory_blocked:
            return i, 0, False
        previous_pose_solutions = current_pose_solutions
        current_pose_solutions = []
    for node in previous_pose_solutions:
        G.add_edge(np.array2string(node), "t")
    min_max_edge_trajectory = min_max_edge_dijkstra(G, "s", "t", "weight")
    if min_max_edge_trajectory == None:
        return i, 0, False
    ik_trajectory_max_edge, graph_ik_trajectory = min_max_edge_trajectory
    ik_trajectory = []
    for idx in range(1, len(graph_ik_trajectory) - 1):
        ik_trajectory.append(np.fromstring(graph_ik_trajectory[idx].strip("[]"), sep=" ", dtype=float))  # type: ignore
    ik_trajectory = adjust_trajectory(np.array(ik_trajectory))
    np.savetxt(
        output_path,
        np.array(ik_trajectory),
        delimiter=",",
        fmt="%.5f",
    )
    return len(placed_trajectory) - 1, ik_trajectory_max_edge, True
