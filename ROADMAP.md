# Roadmap

Each item states the goal, the command that exercises it, and the condition that
must hold for the checkbox to be marked. Commands assume:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

Arm joint order used everywhere in this project:
`shoulder_pan_joint, shoulder_lift_joint, elbow_flex_joint, wrist_flex_joint, wrist_roll_joint`.
`gripper_joint` is handled separately from the `manipulator` planning group.

## 1. Upstream stack and development environment

- [x] Confirm the required SO-101 packages build and run on Ubuntu 26.04 / ROS 2 Lyrical.
  - Run: `rosdep install --from-paths src --ignore-src -r -y`
  - Pass: rosdep resolves every dependency of `ros2_so_arm`, `mujoco_ros2_control`, and `feetech_ros2_driver` with no unresolved keys.
- [x] Record the verified dependency revisions and the native installation steps.
  - Pass: `README.md` lists the exact Ubuntu, ROS 2, MoveIt, ros2_control, MuJoCo vendor versions and the pinned Git revisions of all three imported repositories.
- [x] Track source dependencies in `dependencies.repos` and upstream changes in `patches/`, without committing upstream source.
  - Run: `vcs import src < dependencies.repos && ./scripts/apply_dependency_patches.sh`
  - Pass: the script applies cleanly on a fresh checkout and fails loudly if a repository is at an unexpected revision; `git status` shows no upstream source files staged.
- [x] Build the imported workspace cleanly from empty build/install directories.
  - Run: `colcon build --symlink-install`
  - Pass: all 11 imported packages finish, no package fails.
- [x] Confirm the upstream SO-101 description loads with the expected joint names and a connected TF tree.
  - Run: `python3 scripts/verify_step1_mock.py`
  - Pass: the script reports the 6 expected joint names and one connected TF tree from `world` to every SO-101 link.
  - Interactive equivalent: `ros2 launch so_arm101_moveit_config demo.launch.py hardware_type:=mock_components`

## 2. Validate upstream MuJoCo and standard control path

- [x] Check that the MuJoCo bringup starts without project-specific robot-control code.
  - Run: `ros2 launch so_arm101_moveit_config demo.launch.py hardware_type:=mujoco`
  - Pass: the MuJoCo viewer opens with the SO-101 loaded and no node exits with an error.
- [x] Check that `/joint_states` and TF reflect the MuJoCo simulation state.
  - Run: the MuJoCo bringup above, then `ros2 topic echo /joint_states --once` and `ros2 run tf2_ros tf2_echo world gripper_link`.
  - Pass: `/joint_states` publishes the 5 arm joints plus `gripper_joint` with `use_sim_time` active, and the `world` -> `gripper_link` transform changes consistently when the joints move in MuJoCo.
- [x] Check that the standard arm controller is active and exposes its action interface.
  - Run: `ros2 control list_controllers` and `ros2 action list`
  - Pass: `joint_state_broadcaster`, `joint_trajectory_controller`, and `gripper_controller` are all `active`, and `/joint_trajectory_controller/follow_joint_trajectory` is listed.
- [x] Check that a commanded joint trajectory is actually executed in MuJoCo.
  - Run: send a two-point, time-parameterized `FollowJointTrajectory` goal for the 5 arm joints (for example `manipulator` group state `zero` -> `rest`) with `ros2 action send_goal ... --feedback`.
  - Pass: the goal returns `SUCCEEDED`, and `/joint_states` settles at the commanded final positions within the controller's tolerance. Compilation or process startup alone is not sufficient.
- [ ] Preserve one repeatable MuJoCo bringup/verification command.
  - Pass: a single documented command (script or launch line) reproduces the three checks above, and it is recorded in `README.md`.

## 3. Validate MoveIt and Cartesian capability

- [ ] Check that MoveIt starts against the MuJoCo-controlled SO-101.
  - Run: `ros2 launch so_arm101_moveit_config demo.launch.py hardware_type:=mujoco`
  - Pass: `move_group` reports the `manipulator` and `gripper` groups loaded, and RViz shows the MotionPlanning panel with the robot state matching MuJoCo.
- [ ] Check that a MoveIt-planned joint motion executes through ros2_control in MuJoCo.
  - Run: plan and execute a motion between two named `manipulator` states from RViz or MoveItCpp.
  - Pass: execution reports success, and the final `/joint_states` matches the planned trajectory endpoint.
- [ ] Check MoveIt FK against the simulator and TF.
  - Run: compare the MoveIt FK pose of `gripper_link` (relative to `base_link`) against `ros2 run tf2_ros tf2_echo base_link gripper_link` for several joint configurations.
  - Pass: position and orientation agree within a stated numerical tolerance for every tested configuration.
- [ ] Check that IK can solve position plus tool-axis alignment without constraining tool roll.
  - Run: request IK for a set of target poses that fix the tool position and tool-axis direction but leave roll about that axis free.
  - Pass: the chosen solver returns valid, in-limit solutions for reachable targets, and the FK of each solution reproduces the requested position and tool-axis direction.
- [ ] Check self-collision and environment collision detection.
  - Run: query the planning scene with a known self-colliding configuration, a configuration intersecting an added collision object, and a known collision-free configuration.
  - Pass: the first two are reported as colliding and the third as collision-free.
- [ ] Check a Cartesian waypoint path for continuity and feasibility.
  - Run: plan a short straight-line Cartesian path through several waypoints and execute it in MuJoCo.
  - Pass: the returned path achieves full requested fraction, contains no joint discontinuities between consecutive points, and executes to the final waypoint.

## 4. Surface-following geometry core

- [ ] Create the project-owned `surface_following` package with the minimum structure for simulator-independent geometry plus thin ROS integration.
  - Run: `colcon build --packages-select surface_following && colcon test --packages-select surface_following`
  - Pass: build and the (initially trivial) test suite succeed, and the geometry module imports without ROS or MuJoCo on the path.
- [ ] Define the minimal surface representation that carries 3D points and consistently oriented normals.
  - Pass: unit tests confirm the representation stores points and unit-length normals with a documented outward orientation convention, and rejects inputs that violate it.
- [ ] Implement a planar surface with explicit surface and normal frame conventions.
  - Pass: unit tests confirm point queries and normals for a plane at a known pose, including a non-axis-aligned pose.
- [ ] Generate a deterministic raster/serpentine coverage path over a bounded planar region.
  - Pass: unit tests confirm the sample count, row spacing, in-row spacing, alternating row direction, and that repeated calls return identical output.
- [ ] Convert surface samples into stand-off tool targets with the tool axis on the local normal and tool roll set explicitly.
  - Pass: unit tests confirm each target sits at the requested stand-off distance along the normal, its tool axis is anti-parallel to the surface normal, and roll follows the documented rule.
- [ ] Cover path ordering, spacing, stand-off geometry, normal orientation, and pose continuity with focused tests.
  - Run: `colcon test --packages-select surface_following`
  - Pass: all tests pass with no ROS or MuJoCo dependency, and consecutive target poses differ by less than a stated position and orientation step on a smooth surface.

## 5. Planar end-to-end POC

- [ ] Add a planar work surface to MuJoCo and to the MoveIt planning scene at a consistent pose.
  - Run: the MuJoCo bringup with the surface added, then `ros2 run tf2_ros tf2_echo base_link <surface_frame>`.
  - Pass: the surface appears in both MuJoCo and RViz at the same pose relative to `base_link`, within a stated tolerance.
- [ ] Feed planar stand-off targets into the MoveIt pipeline through a thin adapter.
  - Pass: the adapter converts geometry-core targets into MoveIt pose goals, and the geometry core still passes its tests with no robot-specific imports.
- [ ] Convert raster targets into a collision-checked feasible joint path and report unreachable targets explicitly.
  - Pass: for a reachable raster every target yields a collision-free solution; for a deliberately out-of-reach raster the unreachable targets are named in the output rather than skipped silently.
- [ ] Time-parameterize and execute the full raster through MoveIt and `joint_trajectory_controller`.
  - Pass: execution reports success and `/joint_states` follows the trajectory to the final raster point without controller tolerance violations.
- [ ] Check coverage, stand-off distance, tool-axis alignment, and execution smoothness in simulation.
  - Run: log `world` -> tool TF during execution and compare against the commanded targets.
  - Pass: the executed tool path covers the raster pattern, holds the stand-off distance within a stated tolerance, keeps the tool axis within a stated angle of the surface normal, and shows no velocity discontinuities.
- [ ] Preserve one repeatable planar surface-following demo command.
  - Pass: a single documented launch or run command reproduces the demo end to end, and it is recorded in `README.md`.

## 6. Curved and folded surfaces

- [ ] Add a curved surface representation built from sampled geometry or a mesh.
  - Pass: unit tests confirm point and normal queries against an analytically known curved surface (for example a cylinder) within a stated tolerance.
- [ ] Generate and execute a curved-surface raster with smooth position and normal evolution.
  - Pass: consecutive targets stay within the stated position and orientation step on smooth regions, and the raster executes in MuJoCo with stand-off and alignment held to the same tolerances as the planar case.
- [ ] Add an angled or folded surface with explicit segmentation at normal discontinuities.
  - Pass: unit tests confirm the segmentation splits the surface exactly at the fold, and normals are not smoothed across it.
- [ ] Execute the folded-surface case with collision checking and controlled segment transitions.
  - Pass: execution completes collision-free, and the tool reorients between segments through an explicit transition rather than an unbounded jump in commanded orientation.

## 7. Perception and real robot

- [ ] Add simulated depth/RGB-D perception and convert it into the existing surface representation.
  - Pass: a simulated point cloud of a known plane yields a surface whose pose and normals match ground truth within a stated tolerance.
- [ ] Integrate the Kinect v2 through standard ROS sensor messages and TF, including sensor-to-robot extrinsics.
  - Run: `ros2 topic hz` on the depth topic and `ros2 run tf2_ros tf2_echo base_link <camera_optical_frame>`.
  - Pass: the sensor publishes at its nominal rate and the calibrated extrinsic transform is available in TF.
- [ ] Validate planar surface reconstruction from real sensor data before using it for motion.
  - Pass: the reconstructed plane from real data matches a physically measured plane pose within a stated tolerance.
- [ ] Bring up the real SO-101 through the upstream ros2_control hardware path.
  - Run: `ros2 launch so_arm101_description controllers_bringup.launch.py hardware_type:=real usb_port:=<port>`
  - Pass: the same controller and joint names as the simulated path become active, with live state feedback on `/joint_states`.
- [ ] Validate real joint units, calibration, limits, ordering, state feedback, and safety behavior before any MoveIt execution.
  - Pass: commanded and measured joint values agree in sign, unit, and ordering for each joint individually, limits are enforced, and the stop/safety behavior is demonstrated.
- [ ] Execute the same planar surface-following pipeline on the real SO-101.
  - Pass: the demo runs on hardware with no change to the surface-following mathematics; only robot, control, and MoveIt configuration differ.
- [ ] Demonstrate portability with a second manipulator.
  - Pass: the second robot runs the pipeline through configuration changes only, and the `surface_following` geometry tests pass unchanged.
