# SO-101 Surface Following

Research POC for surface-following with the Hugging Face SO-101 robot.

The project generates raster/serpentine tool paths over 3D surfaces while maintaining stand-off distance and tool-axis alignment with the local surface normal.

## Architecture

```text
surface / perception
        |
        v
surface_following
        |
        v
MoveIt 2 / TF / ROS 2
        |
        v
joint_trajectory_controller
        |
        v
ros2_control
      /      \
 MuJoCo      real SO-101
```

The project reuses upstream robot infrastructure rather than maintaining its own SO-101 stack:

- `ros-physical-ai/ros2_so_arm` for the SO-101 ROS description, MoveIt configuration, ros2_control integration, and robot bringup.
- `ros-controls/mujoco_ros2_control` for the MuJoCo ros2_control backend.

Project-owned code should remain focused on surface representation, coverage-path generation, stand-off/tool-target generation, perception adaptation, and end-to-end surface-following behavior.

## Target environment

- Ubuntu 26.04
- ROS 2 Lyrical
- Ubuntu system Python used by ROS
- Native ROS/Ubuntu dependency management with apt and rosdep
- MuJoCo
- MoveIt 2
- ros2_control / ros2_controllers

The first roadmap milestone verifies the exact compatible upstream revisions before they are pinned.

## Repository layout

```text
so101-surface-following/
├── src/
│   ├── ros2_so_arm/            # external, imported with vcstool
│   ├── mujoco_ros2_control/     # external, imported with vcstool
│   └── surface_following/       # project-owned package, added when its milestone starts
├── dependencies.repos
├── PROJECT_CONTEXT.md
├── ROADMAP.md
├── DEVELOPMENT_GUIDELINES.md
└── README.md
```

Imported upstream repositories should not be committed into this repository. The tracked `dependencies.repos` file defines their source and revision.

## Bootstrap

Install the normal ROS development tools, including `vcstool`, `rosdep`, and `colcon`, then from the repository root:

```bash
mkdir -p src
vcs import src < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
```

After a successful build:

```bash
source install/setup.bash
```

Do not add Conda, Pixi, pyenv, Poetry, or another Python version unless a concrete dependency later requires a different isolation strategy.

## Development workflow

Before implementation work, read:

1. `PROJECT_CONTEXT.md`
2. `ROADMAP.md`
3. `DEVELOPMENT_GUIDELINES.md`

Implement one roadmap milestone at a time, verify it, mark only that milestone complete, and stop before starting the next one.
