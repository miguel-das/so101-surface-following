#!/usr/bin/env python3
"""Verify IK for tool position plus tool-axis alignment with tool roll left free.

Surface following needs the tool tip at a stand-off point with the tool axis on
the local surface normal. Roll about that axis is irrelevant, so it must not be
constrained. That is 5 task constraints (position 3 + axis direction 2) for the
5 joints of the SO-101 `manipulator` group: exactly determined, unlike a full
6-DOF pose goal, which is over-determined and is why full-pose IK and RViz's
"Use Cartesian path" fail on this arm.

Two results drive the implementation and are re-checked here before the IK runs:

  * The tool frame must be `gripper_frame_link`, not the SRDF chain tip
    `gripper_link`. The tool axis is collinear with `wrist_roll_joint`, so
    wrist_roll cannot reorient it. `gripper_link`'s origin also sits on that
    axis, leaving wrist_roll with no effect at all on (position, axis) and the
    task Jacobian rank 4 for 5 constraints -- structurally unsolvable.
    `gripper_frame_link` is offset 7.9 mm off the roll axis, which restores
    rank 5.

  * No IK plugin packaged for ROS 2 Lyrical can express "free roll about a
    given axis": kdl_kinematics_plugin solves the full pose, trac_ik exposes
    only `position_only_ik` (frees all three rotations), and pick_ik exposes
    only a scalar `rotation_scale`. So the axis-alignment task is solved here
    directly, by damped least squares on the 5-constraint residual, using
    MoveIt's own `/compute_fk` as the single source of forward kinematics so no
    second copy of the robot model is introduced.

Run:
    python3 scripts/verify_tool_axis_ik.py

Pass: every reachable target yields an in-limit solution whose FK reproduces the
requested tool position and tool-axis direction within POSITION_TOLERANCE_M and
AXIS_TOLERANCE_RAD, while the same targets with an imposed roll are rejected by
the stock full-pose solver.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Arm joint order used everywhere in this project.
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
]
BASE_FRAME = "base_link"
# The tool frame. See the module docstring: gripper_link cannot work.
TOOL_LINK = "gripper_frame_link"
CHAIN_TIP_LINK = "gripper_link"

# Acceptance tolerances for a solution, and the tighter targets the solver
# itself converges to so that acceptance is not merely reporting the solver's
# own stopping rule.
POSITION_TOLERANCE_M = 1.0e-4
AXIS_TOLERANCE_RAD = 1.0e-3
SOLVER_POSITION_TOLERANCE_M = 1.0e-6
SOLVER_AXIS_TOLERANCE_RAD = 1.0e-6

# Levenberg-Marquardt settings. Adaptive damping matters here because the task
# is badly scaled: the tool-axis directions are controlled through the arm's
# long links while the fifth constraint is carried by a 7.9 mm offset, so the
# task Jacobian's condition number runs to a few hundred.
FD_STEP_RAD = 1.0e-6
INITIAL_DAMPING = 1.0e-3
MIN_DAMPING = 1.0e-9
MAX_DAMPING = 1.0e4
DAMPING_STEP = 3.0
MAX_STEP_RAD = 0.3
MAX_ITERATIONS = 150

# The task is exactly determined, so its solutions are isolated points and a
# local method has to start in the right basin. Rather than restart blindly,
# screen many random configurations by residual (one FK call each, ~0.4 ms) and
# run the expensive local solve only from the most promising ones.
SCREEN_SAMPLES = 400
SEED_ATTEMPTS = 8

# Rank check: the smallest singular value of the 5x5 task Jacobian must clear
# this to count as full rank. The rank-4 case lands ~1e-13, the rank-5 case
# ~5e-3, so the threshold is far from both.
RANK_SINGULAR_VALUE_FLOOR = 1.0e-6

TARGET_COUNT = 12
RANDOM_SEED = 20260904


def rotation_from_quaternion(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quaternion_from_rotation(R: np.ndarray):
    """Return (x, y, z, w) for a proper rotation matrix."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def rotation_about_axis(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """Rodrigues rotation of `angle_rad` about a unit `axis`."""
    kx, ky, kz = axis
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + math.sin(angle_rad) * K + (1.0 - math.cos(angle_rad)) * (K @ K)


def axis_error_vector(current_axis: np.ndarray, target_axis: np.ndarray) -> np.ndarray:
    """Rotation vector taking `current_axis` onto `target_axis`.

    Its magnitude is the angle between them and its direction is perpendicular
    to both, so it carries no component about the tool axis. That is what leaves
    roll unconstrained: the residual simply cannot express a roll error.
    """
    cross = np.cross(current_axis, target_axis)
    sin_angle = float(np.linalg.norm(cross))
    cos_angle = float(np.dot(current_axis, target_axis))
    angle = math.atan2(sin_angle, cos_angle)
    if sin_angle < 1.0e-12:
        # Parallel (no error) or anti-parallel (any perpendicular axis works).
        if cos_angle > 0.0:
            return np.zeros(3)
        helper = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(helper, current_axis)) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        perpendicular = np.cross(current_axis, helper)
        return math.pi * perpendicular / np.linalg.norm(perpendicular)
    return angle * cross / sin_angle


class ToolAxisIK(Node):
    """Forward kinematics from MoveIt, plus the 5-constraint IK built on it."""

    def __init__(self) -> None:
        super().__init__("so101_tool_axis_ik_verifier")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.robot_description: str | None = None
        self.create_subscription(
            String,
            "/robot_description",
            self._on_robot_description,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        for client in (self.fk_client, self.ik_client):
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f"{client.srv_name} never became available")

    def _on_robot_description(self, message: String) -> None:
        self.robot_description = message.data

    def joint_limits_rad(self) -> dict[str, tuple[float, float]]:
        """Read revolute limits straight from the running robot description."""
        deadline = time.monotonic() + 30.0
        while self.robot_description is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.robot_description is None:
            raise RuntimeError("no /robot_description was published")
        root = ET.fromstring(self.robot_description)
        limits: dict[str, tuple[float, float]] = {}
        for joint in root.findall("joint"):
            name = joint.get("name")
            limit = joint.find("limit")
            if name in ARM_JOINTS and limit is not None:
                limits[name] = (float(limit.get("lower")), float(limit.get("upper")))
        missing = set(ARM_JOINTS) - set(limits)
        if missing:
            raise RuntimeError(f"robot description is missing limits for {sorted(missing)}")
        return limits

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        result = future.result()
        if result is None:
            raise RuntimeError(f"{client.srv_name} call timed out")
        return result

    def forward_kinematics(self, joint_positions_rad, links=(TOOL_LINK,)):
        """Return {link: (position 3-vector, rotation 3x3)} in BASE_FRAME."""
        request = GetPositionFK.Request()
        request.header.frame_id = BASE_FRAME
        request.fk_link_names = list(links)
        state = RobotState()
        state.joint_state = JointState(
            name=ARM_JOINTS, position=[float(v) for v in joint_positions_rad]
        )
        request.robot_state = state
        response = self._call(self.fk_client, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"FK failed with code {response.error_code.val}")
        poses = {}
        for name, stamped in zip(response.fk_link_names, response.pose_stamped):
            position = np.array(
                [
                    stamped.pose.position.x,
                    stamped.pose.position.y,
                    stamped.pose.position.z,
                ]
            )
            poses[name] = (position, rotation_from_quaternion(stamped.pose.orientation))
        return poses

    def tool_pose(self, joint_positions_rad, link=TOOL_LINK):
        position, rotation = self.forward_kinematics(joint_positions_rad, [link])[link]
        return position, rotation

    # -- the 5-constraint solve ------------------------------------------------

    def _residual(self, joint_positions_rad, target_position, target_axis):
        position, rotation = self.tool_pose(joint_positions_rad)
        return (
            np.concatenate(
                [target_position - position, axis_error_vector(rotation[:, 2], target_axis)]
            ),
            position,
            rotation[:, 2],
        )

    def solve_position_and_axis(
        self, target_position, target_axis, seed_positions_rad, limits
    ):
        """Damped least squares on [position error (3); axis error (3, rank 2)].

        Roll about the tool axis never appears in the residual, so it is left
        free rather than driven to any particular value.
        """
        lower = np.array([limits[j][0] for j in ARM_JOINTS])
        upper = np.array([limits[j][1] for j in ARM_JOINTS])
        theta = np.clip(np.array(seed_positions_rad, dtype=float), lower, upper)
        identity = np.eye(len(ARM_JOINTS))

        residual, position, axis = self._residual(theta, target_position, target_axis)
        cost = float(residual @ residual)
        damping = INITIAL_DAMPING

        for _ in range(MAX_ITERATIONS):
            if self._within(position, axis, target_position, target_axis):
                return theta

            jacobian = np.zeros((6, len(ARM_JOINTS)))
            for index in range(len(ARM_JOINTS)):
                perturbed = theta.copy()
                perturbed[index] += FD_STEP_RAD
                perturbed = np.clip(perturbed, lower, upper)
                step = perturbed[index] - theta[index]
                if step == 0.0:
                    perturbed[index] = theta[index] - FD_STEP_RAD
                    step = perturbed[index] - theta[index]
                perturbed_residual, _, _ = self._residual(
                    perturbed, target_position, target_axis
                )
                jacobian[:, index] = (perturbed_residual - residual) / step

            normal_matrix = jacobian.T @ jacobian
            gradient = jacobian.T @ residual
            accepted = False
            while damping <= MAX_DAMPING:
                # dq = -(J^T J + lambda I)^-1 J^T r, with lambda raised until the
                # step actually reduces the residual.
                delta = -np.linalg.solve(normal_matrix + damping * identity, gradient)
                largest = float(np.max(np.abs(delta)))
                if largest > MAX_STEP_RAD:
                    delta *= MAX_STEP_RAD / largest
                candidate = np.clip(theta + delta, lower, upper)
                candidate_residual, candidate_position, candidate_axis = self._residual(
                    candidate, target_position, target_axis
                )
                candidate_cost = float(candidate_residual @ candidate_residual)
                if candidate_cost < cost:
                    theta = candidate
                    residual, position, axis = (
                        candidate_residual,
                        candidate_position,
                        candidate_axis,
                    )
                    cost = candidate_cost
                    damping = max(damping / DAMPING_STEP, MIN_DAMPING)
                    accepted = True
                    break
                damping *= DAMPING_STEP
            if not accepted:
                # Damping saturated: this seed cannot make further progress.
                return theta if self._within(position, axis, target_position, target_axis) else None

        return theta if self._within(position, axis, target_position, target_axis) else None

    def solve_with_multistart(self, target_position, target_axis, limits, rng):
        """Screen random configurations by residual, then solve from the best."""
        lower = np.array([limits[j][0] for j in ARM_JOINTS])
        upper = np.array([limits[j][1] for j in ARM_JOINTS])

        candidates = [np.zeros(len(ARM_JOINTS))]
        candidates.extend(rng.uniform(lower, upper) for _ in range(SCREEN_SAMPLES))
        scored = []
        for candidate in candidates:
            residual, _, _ = self._residual(candidate, target_position, target_axis)
            scored.append((float(residual @ residual), candidate))
        scored.sort(key=lambda item: item[0])

        for _, seed in scored[:SEED_ATTEMPTS]:
            solution = self.solve_position_and_axis(
                target_position, target_axis, seed, limits
            )
            if solution is not None:
                return solution
        return None

    @staticmethod
    def _within(position, axis, target_position, target_axis) -> bool:
        position_error = float(np.linalg.norm(target_position - position))
        axis_error = float(np.linalg.norm(axis_error_vector(axis, target_axis)))
        return (
            position_error <= SOLVER_POSITION_TOLERANCE_M
            and axis_error <= SOLVER_AXIS_TOLERANCE_RAD
        )

    # -- the stock full-pose solver, used as the contrast ----------------------

    def solve_full_pose(self, position, rotation, seed_positions_rad, timeout_s=0.5):
        request = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = "manipulator"
        ik.ik_link_name = TOOL_LINK
        state = RobotState()
        state.joint_state = JointState(
            name=ARM_JOINTS, position=[float(v) for v in seed_positions_rad]
        )
        ik.robot_state = state
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = BASE_FRAME
        ik.pose_stamped.pose.position.x = float(position[0])
        ik.pose_stamped.pose.position.y = float(position[1])
        ik.pose_stamped.pose.position.z = float(position[2])
        x, y, z, w = quaternion_from_rotation(rotation)
        ik.pose_stamped.pose.orientation.x = x
        ik.pose_stamped.pose.orientation.y = y
        ik.pose_stamped.pose.orientation.z = z
        ik.pose_stamped.pose.orientation.w = w
        ik.timeout.sec = int(timeout_s)
        ik.timeout.nanosec = int((timeout_s % 1.0) * 1e9)
        ik.avoid_collisions = False
        request.ik_request = ik
        response = self._call(self.ik_client, request)
        return response.error_code.val == MoveItErrorCodes.SUCCESS

    # -- tool frame feasibility -------------------------------------------------

    def task_jacobian_singular_values(self, joint_positions_rad, link):
        """Singular values of d(position, axis)/d(theta), a 5x5 matrix."""
        base_position, base_rotation = self.tool_pose(joint_positions_rad, link)
        base_axis = base_rotation[:, 2]
        helper = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(helper, base_axis)) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        tangent_1 = np.cross(base_axis, helper)
        tangent_1 /= np.linalg.norm(tangent_1)
        tangent_2 = np.cross(base_axis, tangent_1)

        jacobian = np.zeros((5, len(ARM_JOINTS)))
        for index in range(len(ARM_JOINTS)):
            perturbed = np.array(joint_positions_rad, dtype=float)
            perturbed[index] += FD_STEP_RAD
            position, rotation = self.tool_pose(perturbed, link)
            jacobian[0:3, index] = (position - base_position) / FD_STEP_RAD
            axis_delta = (rotation[:, 2] - base_axis) / FD_STEP_RAD
            jacobian[3, index] = float(np.dot(axis_delta, tangent_1))
            jacobian[4, index] = float(np.dot(axis_delta, tangent_2))
        return np.linalg.svd(jacobian, compute_uv=False)


def launch_stack(log_directory: Path):
    environment = os.environ.copy()
    environment.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    processes = []
    for name, arguments in (
        (
            "controllers",
            [
                "ros2", "launch", "so_arm101_description",
                "controllers_bringup.launch.py", "hardware_type:=mock_components",
            ],
        ),
        ("move_group", ["ros2", "launch", "so_arm101_moveit_config", "move_group.launch.py"]),
    ):
        handle = (log_directory / f"{name}.log").open("w")
        processes.append(
            (
                subprocess.Popen(
                    arguments,
                    env=environment,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    text=True,
                ),
                handle,
            )
        )
        time.sleep(8.0)
    return processes


def shut_down(processes):
    for process, handle in processes:
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        handle.close()


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="so101-tool-axis-ik-") as temporary_dir:
        processes = launch_stack(Path(temporary_dir))
        rclpy.init()
        node = ToolAxisIK()
        try:
            return run_checks(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            shut_down(processes)


def run_checks(node: ToolAxisIK) -> int:
    limits = node.joint_limits_rad()
    lower = np.array([limits[j][0] for j in ARM_JOINTS])
    upper = np.array([limits[j][1] for j in ARM_JOINTS])
    # Separate streams so that changing the solver cannot change which targets
    # are generated, keeping runs comparable across edits.
    target_rng = np.random.default_rng(RANDOM_SEED)
    seed_rng = np.random.default_rng(RANDOM_SEED + 1)

    print("IK for tool position + tool-axis direction, roll left free")
    print("=" * 78)
    print(f"group 'manipulator', {len(ARM_JOINTS)} joints: {', '.join(ARM_JOINTS)}")
    print(f"tool frame: {TOOL_LINK}   (SRDF chain tip: {CHAIN_TIP_LINK})")
    print(
        f"tolerances: position <= {POSITION_TOLERANCE_M:.1e} m, "
        f"tool axis <= {AXIS_TOLERANCE_RAD:.1e} rad"
    )

    # -- 1. why the tool frame has to be gripper_frame_link -------------------
    print("\n[1] Task Jacobian rank for the two candidate tool frames")
    probe = np.array([0.30, -0.60, 0.80, -0.40, 0.20])
    rank_ok = True
    for link in (CHAIN_TIP_LINK, TOOL_LINK):
        singular_values = node.task_jacobian_singular_values(probe, link)
        smallest = float(singular_values[-1])
        full_rank = smallest > RANK_SINGULAR_VALUE_FLOOR
        print(
            f"  {link:<20} smallest singular value = {smallest:.3e}  "
            f"-> rank {'5 (solvable)' if full_rank else '4 (NOT solvable)'}"
        )
        if link == TOOL_LINK and not full_rank:
            rank_ok = False
        if link == CHAIN_TIP_LINK and full_rank:
            rank_ok = False
    print(
        "  wrist_roll cannot tilt the tool axis, so only the 7.9 mm offset of\n"
        f"  {TOOL_LINK} off the roll axis makes the task exactly determined."
    )

    # -- 2. reachable targets, built by FK so reachability is guaranteed ------
    print(f"\n[2] Solving {TARGET_COUNT} reachable targets (position + axis, roll free)")
    print(
        f"  {'#':>2}  {'position error':>14}  {'axis error':>12}  "
        f"{'in limits':>9}  {'roll vs source':>14}"
    )
    solved = 0
    all_ok = True
    targets = []
    for index in range(TARGET_COUNT):
        source = target_rng.uniform(lower, upper)
        target_position, source_rotation = node.tool_pose(source)
        target_axis = source_rotation[:, 2]
        targets.append((source, target_position, target_axis, source_rotation))

        solution = node.solve_with_multistart(
            target_position, target_axis, limits, seed_rng
        )

        if solution is None:
            print(f"  {index:>2}  {'NO SOLUTION':>14}")
            all_ok = False
            continue

        position, rotation = node.tool_pose(solution)
        position_error = float(np.linalg.norm(position - target_position))
        axis_error = float(np.linalg.norm(axis_error_vector(rotation[:, 2], target_axis)))
        in_limits = bool(np.all(solution >= lower - 1e-9) and np.all(solution <= upper + 1e-9))
        # How far the achieved roll sits from the roll of the configuration the
        # target was generated from: nonzero shows roll really was not imposed.
        roll_difference = abs(float(solution[4] - source[4]))

        ok = (
            position_error <= POSITION_TOLERANCE_M
            and axis_error <= AXIS_TOLERANCE_RAD
            and in_limits
        )
        all_ok &= ok
        solved += 1
        print(
            f"  {index:>2}  {position_error:>14.3e}  {axis_error:>12.3e}  "
            f"{'yes' if in_limits else 'NO':>9}  {roll_difference:>14.4f}"
            + ("" if ok else "   <-- FAILED")
        )

    # -- 3. the contrast: the same targets with a roll imposed ---------------
    print("\n[3] Same targets given to the stock full-pose solver with an imposed roll")
    print("    (position and axis identical, roll rotated 30 deg -- 6 constraints, 5 joints)")
    imposed_roll_solved = 0
    for source, target_position, target_axis, source_rotation in targets:
        rotated = rotation_about_axis(target_axis, math.radians(30.0)) @ source_rotation
        if node.solve_full_pose(target_position, rotated, source):
            imposed_roll_solved += 1
    print(
        f"  full-pose IK solved {imposed_roll_solved}/{len(targets)} of them"
        + ("" if imposed_roll_solved else "   (as expected: roll cannot be imposed)")
    )

    print("\n" + "=" * 78)
    checks = {
        "tool frame rank check": rank_ok,
        f"all {TARGET_COUNT} reachable targets solved in limits and within tolerance": (
            all_ok and solved == TARGET_COUNT
        ),
        "imposing a roll is rejected by the full-pose solver": imposed_roll_solved == 0,
    }
    for label, passed in checks.items():
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}")
    overall = all(checks.values())
    print(f"\nRESULT: {'PASS' if overall else 'FAIL'}")
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
