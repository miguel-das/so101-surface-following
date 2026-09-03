# Roadmap

## 1. Upstream stack and development environment

- [x] Verify Ubuntu 26.04 / ROS 2 Lyrical compatibility for the required SO-101 paths through `ros2_so_arm`, MoveIt 2, ros2_control, ros2_controllers, MuJoCo, and `mujoco_ros2_control`.
- [x] Record the verified dependency sources/revisions and native Ubuntu/ROS installation steps.
- [x] Add a tracked `dependencies.repos` file and patches, and import external ROS repositories without committing their source into this repository.
- [x] Build the imported upstream workspace cleanly with colcon and resolve dependencies with rosdep.
- [x] Verify the upstream SO-101 description can be visualized in RViz with correct joint names and TF topology using a repeatable acceptance check.

## 2. Validate upstream MuJoCo and standard control path

- [ ] Launch the SO-101 in MuJoCo using the upstream `ros2_so_arm` / `mujoco_ros2_control` integration without project-specific robot-control code.
- [ ] Verify `/joint_states` and robot TF reflect MuJoCo state correctly.
- [ ] Verify the standard arm `joint_trajectory_controller` is active and exposes `FollowJointTrajectory`.
- [ ] Execute and verify a simple time-parameterized joint trajectory through the standard controller interface.
- [ ] Record one repeatable MuJoCo bringup/verification command for the project.

## 3. Validate MoveIt and Cartesian capability

- [ ] Launch the upstream MoveIt configuration against the MuJoCo-controlled SO-101.
- [ ] Verify planned joint motion executes through ros2_control in MuJoCo.
- [ ] Verify representative FK results against TF/MuJoCo state.
- [ ] Validate a MoveIt-compatible IK approach for position plus tool-axis alignment without requiring arbitrary tool roll.
- [ ] Verify self-collision and environment collision checking with representative cases.
- [ ] Plan and execute a simple Cartesian waypoint path with continuous feasible joint solutions.

## 4. Surface-following geometry core

- [ ] Create the project-owned `surface_following` package with the smallest structure needed for simulator-independent geometry and thin ROS integration.
- [ ] Define the minimal simulator-independent surface representation needed to carry/query 3D points and consistently oriented normals.
- [ ] Implement a planar surface representation with explicit surface and normal frame conventions.
- [ ] Generate a deterministic raster/serpentine coverage path over a bounded planar region.
- [ ] Convert surface samples into stand-off tool targets with the tool axis aligned to the local normal and tool roll handled explicitly.
- [ ] Add focused tests for path ordering, spacing, stand-off geometry, normal orientation, and pose continuity without requiring ROS or MuJoCo.

## 5. Planar end-to-end POC

- [ ] Add a planar work surface to MuJoCo and the MoveIt planning scene with a consistent pose and frame relationship.
- [ ] Feed planar stand-off targets into the MoveIt pipeline without introducing robot-specific logic into the geometry core.
- [ ] Convert the raster targets into a collision-checked feasible joint path and report unreachable targets explicitly.
- [ ] Time-parameterize and execute the complete raster through MoveIt and the standard trajectory controller.
- [ ] Verify coverage pattern, stand-off distance, tool-axis alignment, and smooth execution in simulation.
- [ ] Preserve one repeatable planar surface-following demo launch or run command.

## 6. Curved and folded surfaces

- [ ] Add a curved surface representation from sampled geometry or a mesh using established geometry/numerical libraries where appropriate.
- [ ] Generate and execute a curved-surface raster with smooth position and normal evolution on smooth regions.
- [ ] Add an angled or folded surface with explicit segmentation at normal discontinuities.
- [ ] Execute the folded-surface case with collision checking and controlled transitions between surface segments.

## 7. Perception and real robot

- [ ] Add simulated depth/RGB-D perception only when needed to exercise the perception boundary and convert it into the existing surface representation.
- [ ] Integrate Kinect v2 through standard ROS sensor messages and TF, including sensor-to-robot extrinsic calibration.
- [ ] Validate planar surface reconstruction from real sensor data before using it for motion.
- [ ] Bring up the real SO-101 through the upstream ros2_control hardware path while preserving controller-facing joint interfaces and names.
- [ ] Validate real joint units, calibration, limits, ordering, state feedback, and required safety behavior before MoveIt execution.
- [ ] Execute the same planar surface-following pipeline on the real SO-101 without changing the surface-following mathematics.
- [ ] Demonstrate portability with a second manipulator by changing robot/MoveIt/control configuration while keeping the geometry tests and algorithms unchanged.
