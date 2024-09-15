import argparse
from datetime import datetime
import json
from math import pi
import os
import numpy as np
from time import time

from collision_detection import load_urdf
from inverse_kinematics import generate_ik_of_trajectory
from utils.parse_config import parse_config


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Path to the trajectory's config file")
    parser.add_argument(
        "config_file", type=str, help="Path to the trajectory's config file"
    )
    return parser.parse_args()


def save_experiment_report(
    no_of_trajectories: int,
    trajectories_attempted: int,
    times: list[float],
    initial_poses: list[float],
    output_dir: str,
):
    experiment_report = {
        "number_of_trajectories": no_of_trajectories,
        "success_rate": no_of_trajectories / trajectories_attempted,
        "average_time": np.mean(times),
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
    (
        trajectory_file,
        urdf_path,
        dh,
        position_factor,
        dynamic_sub_assembly_relative_rotation,
        theta_offsets,
        ref_point_offset,
        test_AB,
    ) = parse_config(args.config_file)
    output_ts_dir_path = os.path.join(
        "./paths/outputs/", datetime.now().strftime("%d-%m-%Y-%H%M")
    )

    if not os.path.exists(output_ts_dir_path):
        os.makedirs(output_ts_dir_path, exist_ok=True)
    trajectory = np.genfromtxt(
        trajectory_file,
        delimiter=",",
        skip_header=1,
    )
    start_time = time()
    no_of_trajectories = 1
    times = []
    initial_poses = []
    trajectories_attempted = 0
    failure_steps = np.zeros(len(trajectory))
    robot_id = load_urdf(urdf_path)
    while len(times) < no_of_trajectories:
        trajectories_attempted += 1
        trajectory_output_dir_path = os.path.join(
            output_ts_dir_path, f"trajectory_{len(times) + 1}"
        )
        if not os.path.exists(trajectory_output_dir_path):
            os.makedirs(trajectory_output_dir_path, exist_ok=True)
        initial_pose = [
            (np.random.rand() - 0.5) * 1,
            (np.random.rand() - 0.5) * 1,
            (np.random.rand() * 0.7 + 0.2),
            (np.random.rand() - 0.5) * 2 * pi,
            (np.random.rand() - 0.5) * 2 * pi,
            (np.random.rand() - 0.5) * 2 * pi,
        ]
        output_path = os.path.join(trajectory_output_dir_path, "ik_trajectory.csv")
        step_reached, delta, ik_success = generate_ik_of_trajectory(
            trajectory_file,
            dh,
            initial_pose,
            position_factor,
            dynamic_sub_assembly_relative_rotation,
            ref_point_offset,
            theta_offsets,
            robot_id,
            output_path,
            test_AB,
        )
        if ik_success:
            print("succeeded")
            trajectory_report = {
                "time": time() - start_time,
                "initial_pose": initial_pose,
            }
            with open(
                os.path.join(trajectory_output_dir_path, "report.json"), "w"
            ) as json_file:
                json.dump(trajectory_report, json_file, indent=4)

            times.append(time() - start_time)
            initial_poses.append(initial_pose)
            start_time = time()
        else:
            print(step_reached)
            failure_steps[step_reached] += 1

    save_experiment_report(
        no_of_trajectories,
        trajectories_attempted,
        times,
        initial_poses,
        output_ts_dir_path,
    )
    np.save(
        os.path.join(output_ts_dir_path, "failure_steps.npy"),
        failure_steps,
    )
