#!/usr/bin/env python3
"""Launch the SO-101 mock demo and verify its joint states, TF tree, and RViz."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


EXPECTED_JOINTS = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
    "gripper_joint",
}

EXPECTED_LINKS = {
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
    "gripper_frame_link",
    "jaw_link",
}

REQUIRED_RVIZ_MESSAGES = (
    "[rviz2-",
    "OpenGl version:",
    "Loading robot model 'so_arm101'",
    "Ready to take commands for planning group manipulator",
)


class DescriptionVerifier(Node):
    def __init__(self) -> None:
        super().__init__("so101_step1_verifier")
        self.joint_names: set[str] | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _joint_state(self, message: JointState) -> None:
        self.joint_names = set(message.name)


def main() -> None:
    environment = os.environ.copy()
    environment.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")

    with tempfile.TemporaryDirectory(prefix="so101-step1-") as temporary_dir:
        launch_log_path = Path(temporary_dir) / "mock-demo.log"
        with launch_log_path.open("w") as launch_log:
            launch_process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "so_arm101_moveit_config",
                    "demo.launch.py",
                    "hardware_type:=mock_components",
                ],
                env=environment,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )

            rclpy.init()
            verifier = DescriptionVerifier()
            deadline = time.monotonic() + 30.0
            transforms_found: set[str] = set()
            launch_output = ""

            try:
                while time.monotonic() < deadline:
                    if launch_process.poll() is not None:
                        raise RuntimeError(
                            f"Mock demo exited early with code {launch_process.returncode}"
                        )

                    rclpy.spin_once(verifier, timeout_sec=0.1)
                    for link in EXPECTED_LINKS - transforms_found:
                        try:
                            verifier.tf_buffer.lookup_transform(
                                "world", link, rclpy.time.Time()
                            )
                            transforms_found.add(link)
                        except TransformException:
                            pass

                    launch_log.flush()
                    launch_output = launch_log_path.read_text()
                    rviz_ready = all(
                        message in launch_output for message in REQUIRED_RVIZ_MESSAGES
                    )
                    if (
                        verifier.joint_names == EXPECTED_JOINTS
                        and transforms_found == EXPECTED_LINKS
                        and rviz_ready
                    ):
                        break
                else:
                    raise RuntimeError(
                        "Timed out waiting for the complete SO-101 joint state and TF tree. "
                        f"Joints: {verifier.joint_names}; transforms: {sorted(transforms_found)}"
                    )
            finally:
                verifier.destroy_node()
                rclpy.shutdown()
                os.killpg(launch_process.pid, signal.SIGINT)
                try:
                    launch_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(launch_process.pid, signal.SIGTERM)
                    launch_process.wait(timeout=5)

        missing_messages = [
            message for message in REQUIRED_RVIZ_MESSAGES if message not in launch_output
        ]
        if missing_messages:
            raise RuntimeError(
                f"RViz did not complete SO-101 initialization; missing {missing_messages}. "
                f"Launch log: {launch_log_path}"
            )

    print("Verified RViz initialization for robot model 'so_arm101'.")
    print(f"Verified joints: {', '.join(sorted(EXPECTED_JOINTS))}")
    print("Verified one connected TF tree from world to every SO-101 link.")


if __name__ == "__main__":
    main()
