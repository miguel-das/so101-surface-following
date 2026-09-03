# External dependency patches

The repositories in `dependencies.repos` are pinned to upstream commits. Run
`scripts/apply_dependency_patches.sh` immediately after `vcs import` to apply
the project-required changes reproducibly.

## `ros2_so_arm.patch`

Base: `ros-physical-ai/ros2_so_arm@e166df9d51f43b24da9b99047c6c51c306bda74f`

- removes SO-100 packages and keeps the SO-101 stack;
- converts the shared MoveIt configuration to SO-101;
- fixes controller namespacing and ROS 2 Lyrical dependency metadata;
- starts the simulated gripper slightly inside its lower joint limit.

## `mujoco_ros2_control.patch`

Base: `ros-controls/mujoco_ros2_control@e6a6160c471a48609fc3f1f7508d7f571e8fa18e`

- stops forcing the MuJoCo window to the monitor dimensions;
- selects GLFW's X11 backend under WSLg so the window receives decorations.
- avoids copying the optional lidar plugin into an apt-owned directory during
  normal overlay builds; standalone `simulate` installation remains opt-in.
