# Development Guidelines

## Working sequence
- Read `PROJECT_CONTEXT.md`, `ROADMAP.md`, and this file before making changes.
- Inspect the repository and imported upstream dependencies before deciding how to implement the current task.
- Treat the first incomplete roadmap milestone as the current task unless the user explicitly selects another one.
- Implement only the current roadmap milestone.
- Do not start preparation work for later milestones unless it is strictly required for the current one.
- Verify the milestone before marking its checkbox complete.
- Stop after the current milestone is implemented and verified.
- If the milestone cannot be completed, leave it unchecked and state the blocking reason clearly.

## Scope and design
- Prefer the simplest correct implementation that satisfies the current milestone.
- Optimize for clarity, inspectability, and iteration speed rather than platform generality.
- Do not introduce frameworks, plugin systems, registries, factories, or generic abstractions without a present need.
- Do not design for hypothetical robots, sensors, simulators, surfaces, or deployment modes.
- Keep diffs focused on the requested behavior.
- Do not perform unrelated cleanup, formatting, renaming, or refactoring.
- Avoid unnecessary defensive programming for impossible states already excluded by the project contract.
- Fail clearly at meaningful boundaries rather than silently guessing or repairing invalid inputs.

## Upstream dependencies
- Reuse `ros-physical-ai/ros2_so_arm` for SO-101 description, MoveIt, ros2_control integration, and supported robot bringup when it satisfies the current requirement.
- Reuse `mujoco_ros2_control` for the MuJoCo ros2_control backend.
- Do not recreate upstream robot description, MoveIt configuration, controller setup, or hardware integration locally without a demonstrated need.
- Treat imported repositories as external dependencies, not project-owned source.
- Do not perform cosmetic refactors or cleanup inside upstream repositories.
- If an upstream change is required, first determine whether configuration or composition can solve the problem without modifying upstream source.
- If a source patch is genuinely required, keep it minimal, document why it is needed, and pin the dependency revision that the patch applies to.
- Prefer contributing a generally useful fix upstream rather than maintaining a permanent local fork.
- Record source revisions for Git dependencies once a working combination is verified.
- Do not depend on unpinned moving branches for a reproducible milestone once compatibility has been established.

## Environment and dependency management
- Target Ubuntu 26.04 and ROS 2 Lyrical unless a critical upstream incompatibility is demonstrated and explicitly discussed.
- Prefer APIs and package structures that remain portable across supported ROS 2 distributions.
- Use the Ubuntu system Python used by ROS.
- Do not introduce Pixi, Conda, pyenv, Poetry, or a second Python version without a concrete requirement.
- Prefer apt and rosdep for ROS and system dependencies.
- Use a system-Python virtual environment only if a required Python dependency cannot be managed cleanly through the native ROS/Ubuntu environment.
- Use `vcstool` and the tracked `dependencies.repos` file for source dependencies.
- Do not commit imported upstream repository contents into the project repository.

## Established robotics infrastructure
- Prefer mature, maintained robotics and numerical libraries over custom implementations.
- Use ROS 2, MoveIt 2, ros2_control, TF, and standard controllers for the problems they already solve.
- Do not implement a custom trajectory controller when `joint_trajectory_controller` is suitable.
- Do not duplicate joint-state publication when `joint_state_broadcaster` is suitable.
- Do not create a simulation-versus-hardware abstraction above ros2_control.
- Do not create custom ROS messages, services, or actions when standard interfaces fit.
- Prefer an established MoveIt-compatible IK solver when it satisfies the actual kinematic constraints.
- Use existing collision checking, planning-scene, trajectory-processing, and time-parameterization functionality where suitable.
- Use established geometry, point-cloud, mesh, optimization, and linear-algebra libraries instead of reimplementing standard algorithms.

## Project architecture boundaries
- The project-owned `surface_following` package should contain the research/application logic unique to this project.
- Keep surface-following mathematics independent of MuJoCo and ROS message transport where practical.
- Keep ROS adapters thin when the underlying computation can operate on ordinary numerical data structures.
- Do not allow surface-following algorithms to call MuJoCo APIs directly.
- Use MoveIt/TF/standard ROS interfaces as the robot-facing boundary for higher-level application code.
- Use ros2_control as the boundary between the application stack and simulated or real robot hardware.
- Keep simulator-specific project configuration close to the MuJoCo integration rather than in the geometry core.
- Do not split the surface-following package into additional packages solely for theoretical reuse.

## Robot and joint conventions
- Treat the upstream SO-101 description as the robot-specific source unless a documented project requirement overrides it.
- Keep arm joint ordering explicit wherever ordered joint vectors are used.
- Never assume incoming ROS joint-state arrays already match a desired joint order.
- Preserve joint names and semantics across description, ros2_control, controllers, MuJoCo, MoveIt, and tests.
- Treat the gripper separately from the arm planning group unless a milestone explicitly needs coordinated motion.
- Do not propagate hardware-specific or LeRobot-specific command conventions into surface-following mathematics.
- Convert hardware-specific units or conventions at the appropriate hardware/control boundary.

## Frames and transformations
- State source and destination frames for nontrivial transforms.
- Use TF for runtime frame relationships that belong in the ROS frame tree.
- Use established transform libraries rather than hand-written rotation conversions.
- Use one documented transform-composition convention consistently.
- Make surface, robot base, world, sensor, and tool frame relationships explicit.
- Do not mix active/passive rotation interpretations or frame/basis transformations implicitly.
- For tool orientation, distinguish tool-axis alignment from roll about the tool axis.
- Treat discontinuous surface normals explicitly rather than automatically smoothing them away.

## Units and numerical data
- Use SI units internally unless an external hardware/API interface requires another unit.
- Include units in variable names when ambiguity is likely, such as `distance_m`, `speed_rad_s`, or `duration_s`.
- Make known vector and matrix shapes explicit where that prevents ambiguity.
- Document ordered vector semantics when the physical meaning is not obvious from the type.
- Normalize direction vectors and surface normals at a clear ownership boundary.
- Avoid unexplained numerical constants.
- Do not introduce tolerances, smoothing parameters, or thresholds before they are required and justified.
- Keep numerical tests deterministic where practical.

## Naming and comments
- Prefer clear, domain-specific names over abbreviations.
- Name frames, joints, surfaces, trajectories, and transforms unambiguously.
- Avoid generic names such as `data`, `value`, `tmp`, or `result` when a specific meaning is known.
- Comments should explain intent, constraints, conventions, or non-obvious decisions.
- Do not comment obvious syntax or restate code line-by-line.
- Prefer readable direct code over clever compact code.

## ROS and MuJoCo behavior
- Prefer standard message/action types such as `sensor_msgs/msg/JointState`, TF, and `control_msgs/action/FollowJointTrajectory` when applicable.
- Keep controller configuration in configuration files rather than application logic.
- Use `robot_state_publisher` for TF derived from the robot description and joint states.
- Use ROS simulation time consistently when running under MuJoCo.
- Launch files should compose components; they should not contain application algorithms.
- Keep MoveIt configuration robot-specific and surface-following logic robot-agnostic.
- Use MuJoCo-native functionality for simulator-specific concerns.
- Do not expose MuJoCo data structures through ROS-independent application APIs.

## Verification and completion
- Every roadmap milestone must have concrete verification appropriate to the change.
- Prefer automated unit/integration tests for deterministic computation and interfaces.
- Use launch/runtime checks for ROS graph, controller, TF, MoveIt, and MuJoCo integration milestones.
- Verify expected behavior, not merely successful compilation or process startup.
- For motion milestones, verify commanded trajectories are reflected in robot state.
- For geometry milestones, test representative nominal cases and meaningful boundary cases required by that milestone.
- Do not mark a roadmap item complete when verification is skipped, flaky, or only visually assumed without justification.
- Record concise run instructions when a milestone introduces a repeatable demo or validation command.
- Update documentation only where the implemented milestone changes durable project knowledge or usage.
- After verification, mark only the completed roadmap checkbox and stop before the next milestone.
