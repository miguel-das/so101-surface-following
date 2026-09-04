# External dependency patches

The repositories in `dependencies.repos` are pinned to upstream commits. Run
`scripts/apply_dependency_patches.sh` immediately after `vcs import` to apply
the project-required changes reproducibly.

## `ros2_so_arm.patch`

Base: `ros-physical-ai/ros2_so_arm@e166df9d51f43b24da9b99047c6c51c306bda74f`

- removes SO-100 packages and keeps the SO-101 stack;
- converts the shared MoveIt configuration to SO-101;
- fixes controller namespacing and ROS 2 Lyrical dependency metadata;
- uses the MuJoCo simulation clock for robot state publication;
- aligns the MuJoCo gripper range with the URDF so the named open state is reachable;
- gives the arm controller an explicit 0.01 rad goal tolerance and 1 s settling allowance;
- starts the simulated gripper slightly inside its lower joint limit;
- keeps the `rest` named state clear of the joint limits it used to sit on: `elbow_flex_joint` 1.54 -> 1.53 rad (limit 1.54) and `shoulder_lift_joint` -1.745 -> -1.735 rad (limit -1.74533). MuJoCo settles a few 1e-4 rad past a commanded limit, and `CheckStartStateBounds` then rejects every subsequent planning request until `move_group` is restarted.

## `mujoco_ros2_control.patch`

Base: `ros-controls/mujoco_ros2_control@e6a6160c471a48609fc3f1f7508d7f571e8fa18e`

- stops forcing the MuJoCo window to the monitor dimensions;
- selects GLFW's X11 backend under WSLg so the window receives decorations.
- avoids copying the optional lidar plugin into an apt-owned directory during
  normal overlay builds; standalone `simulate` installation remains opt-in.

## `moveit2.patch`

Base: `moveit/moveit2@0b5a5420630ddce212b69c5b5ddef3928783ce26` (2.15.0)

- imports only `moveit_ros/visualization` through Git sparse checkout;
- uses Qt 6 typed combo-box signals so changing the MoveIt planning group
  refreshes and applies its named states.
