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

The versions below were verified together on 2026-09-03:

| Component | Verified version/revision |
| --- | --- |
| Ubuntu | 26.04.1 LTS (`resolute`) |
| ROS 2 | Lyrical |
| MoveIt 2 | `2.15.0` |
| ros2_control / ros2_controllers | `6.9.0` |
| MuJoCo vendor | `0.0.9` |
| `moveit2` visualization base | `0b5a5420630ddce212b69c5b5ddef3928783ce26` (`2.15.0`) |
| `ros2_so_arm` base | `e166df9d51f43b24da9b99047c6c51c306bda74f` |
| `mujoco_ros2_control` base | `e6a6160c471a48609fc3f1f7508d7f571e8fa18e` |
| `feetech_ros2_driver` | `18aed7fb26d3e2b4c0b47762f39d8698b7032422` |

The three patched dependencies are imported at their upstream base revisions and
then modified with the tracked files under `patches/`.

## Repository layout

```text
so101-surface-following/
├── src/
│   ├── ros2_so_arm/            # external, imported with vcstool
│   ├── mujoco_ros2_control/     # external, imported with vcstool
│   ├── feetech_ros2_driver/     # external, imported with vcstool
│   └── surface_following/       # project-owned package
├── patches/                     # reproducible changes to external repositories
├── scripts/
├── dependencies.repos
├── PROJECT_CONTEXT.md
├── ROADMAP.md
├── AGENTS.md
└── README.md
```

Imported upstream repositories should not be committed into this repository. The tracked `dependencies.repos` file defines their source and revision.

## Bootstrap

Install ROS 2 Lyrical and the verified native dependencies:

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-lyrical-desktop \
  ros-lyrical-moveit \
  ros-lyrical-mujoco-vendor \
  ros-lyrical-ros2-control \
  ros-lyrical-ros2-controllers
```

Initialize rosdep once on a new machine, then update its index:

```bash
sudo rosdep init  # omit this line if rosdep is already initialized
rosdep update
```

From a fresh checkout of this repository:

```bash
source /opt/ros/lyrical/setup.bash
mkdir -p src
vcs import src < dependencies.repos
./scripts/apply_dependency_patches.sh
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --allow-overriding moveit_ros_visualization
```

After a successful build:

```bash
source install/setup.bash
```

The patch script intentionally requires the exact revisions from
`dependencies.repos`; it fails instead of applying a patch to an unknown source
revision.

## Step 1 verification

The upstream workspace was verified on Ubuntu 26.04.1 with ROS 2 Lyrical by
running rosdep against all packages and building from empty, isolated build and
install directories. The clean build completed all 11 imported packages. The
generated SO-101 description contains these movable joints:

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_flex_joint
wrist_flex_joint
wrist_roll_joint
gripper_joint
```

The automated acceptance check launches the standard mock hardware demo and
waits for RViz to load `so_arm101`. It then verifies the exact joint-state names
and a transform from `world` to every robot link:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
python3 scripts/verify_step1_mock.py
```

Expected output:

```text
Verified RViz initialization for robot model 'so_arm101'.
Verified joints: elbow_flex_joint, gripper_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_flex_joint, wrist_roll_joint
Verified one connected TF tree from world to every SO-101 link.
```

For interactive inspection, run the underlying demo directly:

```bash
ros2 launch so_arm101_moveit_config demo.launch.py hardware_type:=mock_components
```

Do not add Conda, Pixi, pyenv, Poetry, or another Python version unless a concrete dependency later requires a different isolation strategy.

## Development workflow

Before implementation work, read:

1. `PROJECT_CONTEXT.md`
2. `ROADMAP.md`
3. `DEVELOPMENT_GUIDELINES.md`

Implement one roadmap milestone at a time, verify it, mark only that milestone complete, and stop before starting the next one.
