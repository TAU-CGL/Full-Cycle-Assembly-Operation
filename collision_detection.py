import pybullet as p
import numpy as np

np.set_printoptions(suppress=True, precision=6)
physics_client = p.connect(p.DIRECT)


def check_collision(joint_angles, robot_id):
    num_joints = p.getNumJoints(robot_id)
    for i in range(len(joint_angles)):
        p.resetJointState(robot_id, i, joint_angles[i])

    collision = False
    for i in range(len(joint_angles) - 1):
        for j in range(i + 2, num_joints):
            contact_points = p.getClosestPoints(
                robot_id, robot_id, distance=0, linkIndexA=i, linkIndexB=j
            )
            if len(contact_points) > 0:
                collision = True
                break
        if collision:
            break

    return collision


def load_urdf(urdf_path):
    robot_id = p.loadURDF(urdf_path, useFixedBase=True)
    return robot_id
