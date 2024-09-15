# Experiments Configs

An experiment config supports the following parameters:

- dh - Path to the dh parameters json file.
- path - Path to the trajectory csv file, x,y,z,theta_x,theta_y,theta_z format.
- urdf_path - Path to the urdf file of the robot holding the dynamic sub-assembly, for collision detection.
- dynamic_sub_assembly_relative_rotation - Rotation of the dynamic sub-assembly in relation to the arm's gripper.
- ref_point_offset - Offset from the dynamic sub-assembly origin to the inverse-kinematics TCP.
- theta_offsets - Robotic arm's "home" position adjustments.
- position_factor - Scale factor of the sub-assemblies from the free-flying path planning phase to the robotic arm manipulation path generation.
- test_AB - Boolean, determines whether to test if the path gets too close to the robot's base. If used, d_6 should be moved into the ref_point_offset, and then set to zero (in order for the path to be calculated against the base of the final link).

Config file examples can be found under this directory.
