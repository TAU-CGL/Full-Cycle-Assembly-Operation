import json
import math
from typing import Tuple, List
from numpy.typing import NDArray

import numpy as np


def get_dh_params(file_path: str) -> Tuple[float, ...]:
    with open(file_path, "r") as file:
        data = json.load(file)
    values_tuple: Tuple[float, ...] = tuple(data.values())
    return values_tuple


def parse_config(
    file_path: str,
) -> Tuple[
    str,
    str,
    Tuple[float, ...],
    float,
    List[float],
    NDArray[np.float64],
    NDArray[np.float64],
    bool,
]:
    with open(file_path, "r") as file:
        data = json.load(file)
    dh_path: str = data["dh"]
    path_file: str = data["path"]
    urdf_path: str = data["urdf_path"]
    dynamic_sub_assembly_relative_rotation: List[float] = data[
        "dynamic_sub_assembly_relative_rotation"
    ]
    ref_point_offset: List[float] = data["ref_point_offset"]
    theta_offsets: List[float] = data["theta_offsets"]
    position_factor: float = data["position_factor"]
    test_AB = False
    if "test_AB" in data:
        test_AB = data["test_AB"]
    dh: Tuple[float, ...] = get_dh_params(dh_path)
    dynamic_sub_assembly_relative_rotation[-3:] = list(
        map(math.radians, dynamic_sub_assembly_relative_rotation[-3:])
    )
    return (
        path_file,
        urdf_path,
        dh,
        position_factor,
        dynamic_sub_assembly_relative_rotation,
        np.radians(np.array(theta_offsets)),
        np.array(ref_point_offset),
        test_AB,
    )
