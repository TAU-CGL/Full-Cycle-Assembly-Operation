# Full-Cycle Assembly Operation: From Digital Planning to Trajectory Execution by a Robot Arm

<div>
<img src="https://github.com/TAU-CGL/Full-Cycle-Assembly-Operation/blob/main/results/16505_full_solution.gif?raw=true" height="200" alt="Results16505" style="float: left;">
<img src="https://github.com/TAU-CGL/Full-Cycle-Assembly-Operation/blob/main/results/az_full_size_x4_silent.gif?raw=true" height="200" alt="ResultsAZ" style="float: left;">
<img src="https://github.com/TAU-CGL/Full-Cycle-Assembly-Operation/blob/main/results/UR5e_solving_the_alpha_puzzle.gif?raw=true" height="200" alt="ResultsAlpha" style="float: left;">
</div>

Implementation of our paper, "Full-Cycle Assembly Operation: From Digital Planning to Trajectory Execution by a Robot Arm".

## Abstract

We present an end-to-end framework for planning tight assembly operations, where the input is digital models,
and the output is a full execution plan for a physical robotic arm,
including the trajectory placement and the grasping.
The framework builds on our earlier results on tight assembly planning
for free-flying objects and includes the following novel components:
(i) post processing of the free-flying paths to relax the tightness (where possible) and smooth the path,
(ii) employing analytic path-wise inverse kinematic (IK) solutions with IK-branch switching where needed,
(iii) trajectory placement search based on the above path-wise IK, to determine feasible arm paths, and
(iv) coping with the grasping challenge.
The framework provides guarantees as to the quality of the outcome trajectory.
For each component we provide the algorithmic details and
a full open-source software package for reproducing the process.
Lastly, we demonstrate the framework with tight and challenging assembly problems
(as well as puzzles, which are planned to be hard to assemble),
using a UR5e robotic arm in the real world and in simulation.
See figure at the top for a physical UR5e assembling the alpha-z puzzle
(known to be considerably more complicated to assemble than the celebrated alpha puzzle).
Full video clips of all the assembly demonstrations together with our open source software are available at:
https://github.com/TAU-CGL/Full-Cycle-Assembly-Operation

## Setup

1.  Install dependencies:

        pip install -r requirements.txt

2.  Download and install coppeliaSim from [here](https://coppeliarobotics.com/).

## Usage

The input for our algorithm is a trajectory csv file in x,y,z,theta_x,theta_y,theta_z format, and a urdf file of the robot holding the dynamic sub-assembly, for collision detection. The steps to perform an experiment are:

1. Import the trajectory csv file and the urdf file.
2. Create or use an existing config file, examples and documentation can be found under `experiments`.
3. Create a scene in coppeliaSim for the experiments, examples can be found under `coppeliaSimScenes`.
4. Run `python main.py path/to/config.json`.
5. Results will be created under `outputs`. Copy the path to your trajectory.csv into the associated script in the scene (attached to the robot's object) and play the simulation.

## Free-flying objects path

If you wish to use the TR-RRT algorithm for generating the input trajectory please follow the steps below.

1. Visit https://github.com/TAU-CGL/tr-rrt-public and follow the instructions
2. Add the files from free_flying_post_processing/ into tr-rrt-public/scripts
3. After creating the path, run the two added scripts
