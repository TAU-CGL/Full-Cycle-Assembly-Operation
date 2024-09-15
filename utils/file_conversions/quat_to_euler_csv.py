import numpy as np
import math


def normalize_vector3(v):
    l = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if l == 0:
        return np.array([]), l
    v[0] /= l
    v[1] /= l
    v[2] /= l
    return v, l


def normalize_matrix(matrix):
    axis2, l2 = normalize_vector3(matrix[:, 2])
    if l2 == 0:
        matrix = np.eye(3)
    else:
        axis1, l1 = normalize_vector3(np.cross(axis2, matrix[:, 0]))
        if l1 == 0.0:
            axis1, l1 = normalize_vector3(np.cross(axis2, np.random.rand(3)))
        axis0, l0 = normalize_vector3(np.cross(axis1, axis2))
        matrix = np.column_stack((axis0, axis1, axis2))
    return matrix


def normalize_q(q):
    l = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    q[0] /= l
    q[1] /= l
    q[2] /= l
    q[3] /= l
    return q


def quaternion_to_matrix(w, x, y, z):
    data = normalize_q(np.array([w, x, y, z]))
    print(data, [w, x, y, z])
    retM = np.zeros((3, 3))
    xx = data[1] * data[1]
    xy = data[1] * data[2]
    xz = data[1] * data[3]
    xw = data[1] * data[0]
    yy = data[2] * data[2]
    yz = data[2] * data[3]
    yw = data[2] * data[0]
    zz = data[3] * data[3]
    zw = data[3] * data[0]

    retM[0, 0] = 1.0 - 2.0 * (yy + zz)
    retM[0, 1] = 2.0 * (xy - zw)
    retM[0, 2] = 2.0 * (xz + yw)
    retM[1, 0] = 2.0 * (xy + zw)
    retM[1, 1] = 1.0 - 2.0 * (xx + zz)
    retM[1, 2] = 2.0 * (yz - xw)
    retM[2, 0] = 2.0 * (xz - yw)
    retM[2, 1] = 2.0 * (yz + xw)
    retM[2, 2] = 1.0 - 2.0 * (xx + yy)
    return retM


def get_euler_angles_from_matrix(matrix):
    matrix = normalize_matrix(matrix)
    retV = [0.0, 0.0, 0.0]
    m02 = matrix[0, 2]
    if m02 > 1.0:
        m02 = 1.0
    if m02 < -1.0:
        m02 = -1.0

    retV[1] = math.asin(m02)
    if m02 < 0.0:
        m02 = -m02
    if m02 < 0.999995:
        retV[0] = math.atan2(-matrix[1, 2], matrix[2, 2])
        retV[2] = math.atan2(-matrix[0, 1], matrix[0, 0])
    else:
        retV[0] = 0.0
        retV[2] = math.atan2(matrix[1, 0], matrix[1, 1])
    return retV


def handle_csv(trajectory_file):
    trajectory = np.genfromtxt(
        trajectory_file,
        delimiter=",",
        skip_header=1,
    )
    out = []
    for point in trajectory:
        print(point)
        mat = quaternion_to_matrix(point[3], point[4], point[5], point[6])
        print(mat)
        euler = get_euler_angles_from_matrix(mat)
        out.append([point[0], point[1], point[2], euler[0], euler[1], euler[2]])
    np.savetxt(
        "paths/inputs/industrial_06397/euler_06397_inflated_path.csv",
        np.array(out),
        delimiter=",",
        fmt="%.10f",
    )


handle_csv("paths/inputs/industrial_06397/converted_06397_inflated_path.csv")
