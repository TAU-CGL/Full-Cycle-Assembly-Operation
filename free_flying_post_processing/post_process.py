import os
import json
import time
import torch
import pickle
import argparse

from taucgl_rrt.cspace import SE3
from taucgl_rrt.utils import TEMP_DIR
from taucgl_rrt.visualize import BulletVisualization
from taucgl_rrt.extend import retract, retract_out
from taucgl_rrt.collision_detection import CollisionDetector

parser = argparse.ArgumentParser()
parser.add_argument('--mesh-path', type=str, default="./resources/models/general/06397")
# parser.add_argument('--mesh-path', type=str, default="./resources/models/puzzle/abc_light")

parser.add_argument('--eta', type=float, default=0.002, required=False)
parser.add_argument('--threshold', type=float, default=-0.002, required=False)
# clearance is used for separating the parts in the post-processing
# as the scene is scaled down x10, 0.001 represents 10e-4 meters
parser.add_argument('--clearance', type=float, default=0.001, required=False)
parser.add_argument('--steps', type=int, default=50, required=False)
parser.add_argument('--gui', type=bool, default=True, required=False)
parser.add_argument('--sampling', type=int, default=10000, required=False)
parser.add_argument('--sdf-N', type=int, default=12, required=False)
parser.add_argument('--sdf-H', type=int, default=64, required=False)
parser.add_argument('--device', type=str, default=None, required=False)
args = parser.parse_args()

if args.device is None:
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
else:
    device = torch.device(args.device)


def retract_all(path, cd, clearance, bv):
    retracted_path = []
    path_len = len(path)
    penetrating_configurations = 0
    successfully_retracted = 0
    # for configuration in tqdm(path):
    for configuration in path:
        bv.set_object_configuration("m2", configuration)
        bv.step()
        original_dist = cd.distance_between_objects(configuration)
        print('{}) {:.4}'.format(penetrating_configurations, original_dist), end=' ')
        if original_dist < clearance:
            penetrating_configurations += 1
            new_config = retract_out(cd, configuration, args.eta, clearance,50)
            bv.set_object_configuration("m2", new_config)
            bv.step()
            new_dist = cd.distance_between_objects(new_config)
            if new_dist >= 0:
                successfully_retracted += 1
            retracted_path.append(new_config)
            seperator = ' ' if new_dist >= 0 else '         | '
            print('>>', seperator, '{:.4}'.format(new_dist))
        else:
            retracted_path.append(configuration)
            print('unchanged', '{:.4}'.format(original_dist))
    print('retracted {} configurations out of {} penetrating configurations'.format(successfully_retracted, penetrating_configurations))
    print('penetrating configurations remained / all configurations = ', (penetrating_configurations - successfully_retracted) / path_len)
    return retracted_path


def inflate_path(path, factor):
    new_path = []
    for i in range(len(path)-1):
        qs = SE3.linspace(path[i], path[i+1], factor)
        new_path.extend(qs[:-1])
    new_path.append(path[-1])
    return new_path


def shorten_path(path):
    # skip configurations as long as when paving the new edge, there is either no penetration,
    # or the maximal penetration is no deeper than in the original two edges

    # each iteration tries to drop every other configuration. 'gap' is the distance between the configurations
    # in the original settings
    for gap in [2, 4, 8]:
        length = len(path)
        new_path = []
        print('length before shorten_len: {}'.format(length))
        start = 0
        while start < length - 2:
            print(gap, length, start)
            new_path.append(path[start])
            qs = SE3.linspace(path[start], path[start+2], gap)
            gap_to_beat = min(0.0, cd.distance_between_objects(path[start + 2]),
                                   cd.distance_between_objects(path[start]))
            drop_config = True
            for q in qs[1: -1]:
                q_clearance = cd.distance_between_objects(q)
                condition = float(q_clearance) < gap_to_beat
                if condition:
                    drop_config = False
                    break

            if not drop_config:
                new_path.append(path[start+1])
            start += 2
        new_path.extend(path[start:])
        path = new_path

    print('length after shorten_len: {}'.format(len(new_path)))
    return new_path


if __name__ == "__main__":
    m1_path = os.path.join(args.mesh_path, '0.obj')
    m2_path = os.path.join(args.mesh_path, '1.obj')
    with open(os.path.join(args.mesh_path, 'properties.json'), 'r') as fp:
        properties = json.load(fp)

    q1 = SE3(*properties['q1'])
    q2_start = SE3(*properties['q2_start'])
    q2_end = SE3(*properties['q2_end'])
    if 'sampling' in properties:
        args.sampling = properties['sampling']

    bv = BulletVisualization(gui=args.gui)

    bv.add_object(m1_path, "m1")
    bv.set_object_color("m1", [215 / 256.0, 215 / 256.0, 215 / 256.0, 1])  # silver
    bv.set_object_configuration("m1", q1)

    bv.add_object(m2_path, "m2")
    bv.set_object_color("m2", [201 / 256.0, 176 / 256.0, 55 / 256.0, 1])  # gold
    bv.set_object_configuration("m2", q2_start)

    cd = CollisionDetector(args.sdf_N, args.sdf_H, device)
    cd.set_robot(m2_path)
    cd.generate_robot_sampling(args.sampling)
    cd.set_obstacles(m1_path.replace('.obj', '.pth'), q1)

    head, puzzle_name = os.path.split(args.mesh_path)
    path_filename = os.path.join(TEMP_DIR, puzzle_name + "_path.pkl")
    short_path_filename = os.path.join(TEMP_DIR, puzzle_name + "_short_path.pkl")
    inflated_path_filename = os.path.join(TEMP_DIR, puzzle_name + "_inflated_path.pkl")

    print(puzzle_name)
    with open(path_filename, 'rb') as fp:
        print('Phase 1: retract all')
        path = pickle.load(fp)
        print('path length: {}'.format(len(path)))
        path = retract_all(path, cd, args.clearance, bv)
        inflated_path = inflate_path(path, 2)

        print('Phase 2: shorten path')
        print('path length: {}'.format(len(path)))
        short_path = shorten_path(path)
        print('new path length: {}'.format(len(short_path)))

        with open(short_path_filename, 'wb') as fp:
            pickle.dump(short_path, fp)

        with open(inflated_path_filename, 'wb') as fp:
            pickle.dump(inflated_path, fp)

    frame = 0
    while True:
        bv.set_object_configuration("m2", path[frame])
        bv.step()
        next_frame = (frame + 1) % len(path)
        dist = path[frame].distance(path[next_frame])
        speedup = 0.2
        time.sleep(dist * speedup * 0.05)

        frame = next_frame
        if frame == 0:
            time.sleep(2)
