import argparse
from datetime import datetime
import json
import os
import numpy as np
from math import cos, sin, atan2, sqrt, pi
from typing import Tuple, List
from numpy.typing import NDArray
from time import time

from collision_detection import check_collision_of_path
from utils.parse_config import parse_config
from utils.utils import (
    euler_to_matrix,
    get_euler_angles_from_matrix,
    get_transformation_matrix,
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Path to the path's config file")
    parser.add_argument("config_file", type=str, help="Path to the path's config file")
    return parser.parse_args()


np.set_printoptions(suppress=True, precision=6)


def get_ijk_params(
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
) -> Tuple[float, float, float, float, float, float] | None:
    a_2, a_3, d_1, d_4, d_5, d_6 = dh
    i, j, k = get_ijk_params(shoulder_left, wrist_rh, elbow_up)
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
        theta_1 - pi / 2,
        theta_2 + pi / 2,
        theta_3,
        theta_4 + pi / 2 - pi * 2,
        theta_5,
        (((theta_6 - pi / 2) / pi * 180) % 360) * pi / 180,
    )


def generate_ik_of_path(
    ff_path: str,
    dh: Tuple[float, ...],
    initial_pose: List[float],
    position_factor: float,
    dynamic_alpha_relative_rotation: List[float],
) -> Tuple[int, bool, np.array]:

    dynamic_alpha_relative_rotation_mat = euler_to_matrix(
        dynamic_alpha_relative_rotation[0],
        dynamic_alpha_relative_rotation[1],
        dynamic_alpha_relative_rotation[2],
    )
    init_rot_matrix = np.dot(
        euler_to_matrix(initial_pose[3], initial_pose[4], initial_pose[5]),
        np.linalg.inv(
            euler_to_matrix(
                1.627014, 0.093784, 0.823147
            )  # inverse of initial dynamic alpha angle relative to static alpha
        ),
    )
    inv_init_rot_matrix = np.linalg.inv(init_rot_matrix)
    ff_path[:, :3] *= position_factor
    ff_path[:, :3] = np.dot(ff_path[:, :3] - ff_path[0, :3], inv_init_rot_matrix)
    offset = initial_pose[:3] - ff_path[0, :3]
    ff_path[:, :3] += offset
    offset_path = ff_path
    for i in range(len(offset_path)):
        original = offset_path[i, 3:]
        mat = euler_to_matrix(original[0], original[1], original[2])
        rotated_mat = np.dot(
            np.dot(init_rot_matrix, mat),
            np.linalg.inv(dynamic_alpha_relative_rotation_mat),
        )
        rotated_euler = np.array(get_euler_angles_from_matrix(rotated_mat))
        offset_path[i, 3:] = rotated_euler
    save_attempted_path(np.copy(offset_path), dynamic_alpha_relative_rotation_mat)
    max_dists_prev_angle = [-1, -1, -1, -1, -1, -1]
    ik_path = []
    for i in range(len(offset_path)):
        T = get_transformation_matrix(
            offset_path[i, 0],
            offset_path[i, 1],
            offset_path[i, 2],
            offset_path[i, 3],
            offset_path[i, 4],
            offset_path[i, 5],
        )
        ik = get_inverse_kinematics(dh, T, False, True, True)
        if ik == None:
            return i, False, np.array([])
        ik = np.array(ik)
        current_ik = []
        for j in range(len(ik)):
            if max_dists_prev_angle[j] != -1:
                while abs((ik[j] / pi * 180) - max_dists_prev_angle[j]) > abs(
                    (ik[j] / pi * 180 + 360) - max_dists_prev_angle[j]
                ):
                    ik[j] += pi * 2
                while abs((ik[j] / pi * 180) - max_dists_prev_angle[j]) > abs(
                    (ik[j] / pi * 180 - 360) - max_dists_prev_angle[j]
                ):
                    ik[j] -= pi * 2
            max_dists_prev_angle[j] = ik[j] / pi * 180
            current_ik.append(ik[j])
        ik_path.append(current_ik)
    print(np.array(ik_path)[1] * 180 / pi)
    return len(offset_path) - 1, True, ik_path


def save_attempted_path(
    path: NDArray[np.float64], dynamic_alpha_relative_rotation_mat: NDArray[np.float64]
) -> None:
    for i in range(len(path)):
        original = path[i, 3:]
        mat = euler_to_matrix(original[0], original[1], original[2])
        rotated_mat = np.dot(
            mat,
            dynamic_alpha_relative_rotation_mat,
        )
        rotated_euler = np.array(get_euler_angles_from_matrix(rotated_mat))
        path[i, 3:] = rotated_euler
    np.savetxt(
        "./paths/outputs/attempted_path.csv",
        np.array(path),
        delimiter=",",
        fmt="%.8f",
    )


def save_experiment_report(
    no_of_paths: int,
    times: list[float],
    failed_paths: list[int],
    initial_poses: list[float],
    output_dir: str,
):
    experiment_report = {
        "number_of_paths": no_of_paths,
        "average_time": np.mean(times),
        "average_failed_paths": np.mean(failed_paths),
    }

    with open(os.path.join(output_dir, "experiment_report.json"), "w") as json_file:
        json.dump(experiment_report, json_file, indent=4)

    np.savetxt(
        os.path.join(output_dir, "initial_poses.csv"),
        np.array(initial_poses),
        delimiter=",",
        fmt="%.8f",
    )


if __name__ == "__main__":
    args = parse_arguments()
    path_file, dh, initial_pose, position_factor, dynamic_alpha_relative_rotation = (
        parse_config(args.config_file)
    )
    output_ts_dir_path = os.path.join(
        "./paths/outputs/", datetime.now().strftime("%d-%m-%Y-%H%M")
    )

    if not os.path.exists(output_ts_dir_path):
        os.makedirs(output_ts_dir_path, exist_ok=True)
    ff_path = np.genfromtxt(
        path_file,
        delimiter=",",
        skip_header=1,
    )[:-1]

    start_time = time()
    no_of_paths = 1
    times = []
    initial_poses = []
    failed_paths = []
    failure_steps = np.zeros(len(ff_path))
    current_failed_paths = 0
    while len(times) < no_of_paths:
        path_output_dir_path = os.path.join(
            output_ts_dir_path, f"path_{len(times) + 1}"
        )
        # if not os.path.exists(path_output_dir_path):
        #     os.makedirs(path_output_dir_path, exist_ok=True)
        initial_pose = [
            (np.random.rand() - 0.5) * 2,
            (np.random.rand() - 0.5) * 2,
            (np.random.rand() * 0.5 + 0.2),
            (np.random.rand() - 0.5) * 2 * pi,
            (np.random.rand() - 0.5) * 2 * pi,
            (np.random.rand() - 0.5) * 2 * pi,
        ]
        # output_path = os.path.join(path_output_dir_path, "ik_path.csv")
        # ff_path = np.genfromtxt(
        #     path_file,
        #     delimiter=",",
        #     skip_header=1,
        # )[:-1]

        n_configs, ik_success, joints_path = generate_ik_of_path(
            ff_path,
            dh,
            initial_pose,
            position_factor,
            dynamic_alpha_relative_rotation,
        )

        if ik_success:
            collision_free, step_reached = check_collision_of_path(joints_path)
            if collision_free:
                path_report = {
                    "time": time() - start_time,
                    "failed_paths": current_failed_paths,
                    "initial_pose": initial_pose,
                }
                path_output_dir_path = os.path.join(
                    output_ts_dir_path, f"path_{len(times) + 1}"
                )
                if not os.path.exists(path_output_dir_path):
                    os.makedirs(path_output_dir_path, exist_ok=True)
                output_path = os.path.join(path_output_dir_path, "ik_path.csv")
                np.savetxt(
                    output_path,
                    np.array(joints_path),
                    delimiter=",",
                    fmt="%.5f",
                )
                with open(
                    os.path.join(path_output_dir_path, "report.json"), "w"
                ) as json_file:
                    json.dump(path_report, json_file, indent=4)

                times.append(time() - start_time)
                failed_paths.append(current_failed_paths)
                initial_poses.append(initial_pose)
                start_time = time()
                current_failed_paths = 0
                if len(times) % 20 == 0:
                    save_experiment_report(
                        no_of_paths,
                        times,
                        failed_paths,
                        initial_poses,
                        output_ts_dir_path,
                    )
                    np.save(
                        os.path.join(output_ts_dir_path, "failure_steps.npy"),
                        failure_steps,
                    )
            else:
                failure_steps[step_reached] += 1
                current_failed_paths += 1

    save_experiment_report(
        no_of_paths, times, failed_paths, initial_poses, output_ts_dir_path
    )
    np.save(
        os.path.join(output_ts_dir_path, "failure_steps.npy"),
        failure_steps,
    )
