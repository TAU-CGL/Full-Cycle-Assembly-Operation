from heapq import heappop, heappush
from itertools import count
import math
import numpy as np
from math import cos, sin
from numpy.typing import NDArray
import networkx as nx


def normalize_q(q: NDArray[np.float64]) -> NDArray[np.float64]:
    l = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    q[0] /= l
    q[1] /= l
    q[2] /= l
    q[3] /= l
    return q


def normalize_vector3(v: NDArray[np.float64]) -> tuple[NDArray[np.float64], float]:
    l = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if l == 0:
        return np.array([]), l
    v[0] /= l
    v[1] /= l
    v[2] /= l
    return v, l


def normalize_matrix(matrix: NDArray[np.float64]) -> NDArray[np.float64]:
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


def quaternion_to_matrix(w: float, x: float, y: float, z: float) -> NDArray[np.float64]:
    data = normalize_q(np.array([w, x, y, z]))
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


def euler_to_matrix(a: float, b: float, g: float) -> NDArray[np.float64]:
    retM = np.zeros((3, 3))
    A = cos(a)
    B = sin(a)
    C = cos(b)
    D = sin(b)
    E = cos(g)
    F = sin(g)
    AD = A * D
    BD = B * D
    retM[0, 0] = C * E
    retM[0, 1] = -C * F
    retM[0, 2] = D
    retM[1, 0] = BD * E + A * F
    retM[1, 1] = -BD * F + A * E
    retM[1, 2] = -B * C
    retM[2, 0] = -AD * E + B * F
    retM[2, 1] = AD * F + B * E
    retM[2, 2] = A * C
    return retM


def get_euler_angles_from_matrix(matrix: NDArray[np.float64]) -> list[float]:
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


def get_transformation_matrix(
    x: float, y: float, z: float, theta_x: float, theta_y: float, theta_z: float
) -> NDArray[np.float64]:
    ### experimental adjustments, should check maybe different "home" position for the robot
    x *= -1
    y *= -1
    theta_x *= -1
    theta_y *= -1
    ###

    R = euler_to_matrix(theta_x, theta_y, theta_z)
    t = np.array([x, y, z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T


def min_max_edge_dijkstra(G, source, target, weight_key="weight"):
    if source not in G or target not in G:
        msg = f"Either source {source} or target {target} is not in G"
        raise nx.NodeNotFound(msg)

    if source == target:
        return (0, [source])

    weight = lambda u, v, data: data.get(weight_key, 0)
    push = heappush
    pop = heappop
    # Init:  [Forward, Backward]
    dists = [{}, {}]  # dictionary of final distances
    paths = [{source: [source]}, {target: [target]}]  # dictionary of paths
    fringe = [[], []]  # heap of (distance, node) for choosing node to expand
    seen = [{source: 0}, {target: 0}]  # dict of distances to seen nodes
    c = count()
    # initialize fringe heap
    push(fringe[0], (0, next(c), source))
    push(fringe[1], (0, next(c), target))
    # neighs for extracting correct neighbor information
    if G.is_directed():
        neighs = [G._succ, G._pred]
    else:
        neighs = [G._adj, G._adj]
    # variables to hold shortest discovered path
    # finaldist = 1e30000
    finalpath = []
    dir = 1
    while fringe[0] and fringe[1]:
        # choose direction
        # dir == 0 is forward direction and dir == 1 is back
        dir = 1 - dir
        # extract closest to expand
        (dist, _, v) = pop(fringe[dir])
        if v in dists[dir]:
            # Shortest path to v has already been found
            continue
        # update distance
        dists[dir][v] = dist  # equal to seen[dir][v]
        if v in dists[1 - dir]:
            # if we have scanned v in both directions we are done
            # we have now discovered the shortest path
            return (finaldist, finalpath)

        for w, d in neighs[dir][v].items():
            # weight(v, w, d) for forward and weight(w, v, d) for back direction
            cost = weight(v, w, d) if dir == 0 else weight(w, v, d)
            if cost is None:
                continue
            vwLength = max(dists[dir][v], cost)
            if w in dists[dir]:
                if vwLength < dists[dir][w]:
                    raise ValueError("Contradictory paths found: negative weights?")
            elif w not in seen[dir] or vwLength < seen[dir][w]:
                # relaxing
                seen[dir][w] = vwLength
                push(fringe[dir], (vwLength, next(c), w))
                paths[dir][w] = paths[dir][v] + [w]
                if w in seen[0] and w in seen[1]:
                    # see if this path is better than the already
                    # discovered shortest path
                    totaldist = max(seen[0][w], seen[1][w])
                    if finalpath == [] or finaldist > totaldist:
                        finaldist = totaldist
                        revpath = paths[1][w][:]
                        revpath.reverse()
                        finalpath = paths[0][w] + revpath[1:]
    return None
