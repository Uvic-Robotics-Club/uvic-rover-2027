#!/usr/bin/env python3
"""
joint_cmd_cli.py - Interactive CLI for commanding rover arm joints.

Input format:
    <joint_index> <degrees>

Degrees are RELATIVE to the current joint position — the backend
(SimBackend or CANBackend) tracks actual position and applies the delta.

Example:
    0 45    → rotate turret_joint 45 degrees from current position
    0 45    → rotate another 45 degrees (now at 90 total)
    0 -45   → rotate back 45 degrees (now at 45 total)
    home    → move all joints to home position (0 degrees)
    quit    → exit

Joint index map:
    0 = turret_joint
    1 = shoulder_joint
    2 = elbow_joint
    3 = wrist_pitch_joint
    4 = wrist_roll_joint
    5 = hand_joint
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


JOINT_NAMES = [
    "turret_joint",       # 0
    "shoulder_joint",     # 1
    "elbow_joint",        # 2
    "wrist_pitch_joint",  # 3
    "wrist_roll_joint",   # 4
    "hand_joint",         # 5
]

NUM_JOINTS = len(JOINT_NAMES)


def print_help():
    print("\n--- Rover Arm Joint Command CLI ---")
    print("Input format: <joint_index> <degrees>  (relative move)")
    print("Commands:")
    print("  home      move all joints to home position")
    print("  help      show this message")
    print("  quit      exit")
    print("\nJoint index map:")
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {i} = {name}")
    print()


class JointCommandCLI(Node):

    def __init__(self):
        super().__init__('joint_cmd_cli')

        # Relative commands — backend applies the delta to its own tracked position
        self.relative_publisher = self.create_publisher(
            JointState,
            '/arm/cmd_joint_relative',
            10
        )

        # Absolute commands — only used for 'home'
        self.absolute_publisher = self.create_publisher(
            JointState,
            '/arm/cmd_joint',
            10
        )

        self.get_logger().info("Joint command CLI ready — publishing relative deltas to /arm/cmd_joint_relative")

    def send_command(self, joint_index: int, degrees: float):
        """Publish a relative delta — backend adds this to its own current position."""
        delta_rad = math.radians(degrees)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [JOINT_NAMES[joint_index]]
        msg.position = [delta_rad]  # this is a DELTA, not absolute

        self.relative_publisher.publish(msg)
        self.get_logger().info(
            f"Commanded {JOINT_NAMES[joint_index]}: delta {degrees:.1f}° ({delta_rad:.4f} rad)"
        )

    def send_home(self):
        """Publish absolute home command — all joints to 0."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [0.0] * NUM_JOINTS

        self.absolute_publisher.publish(msg)
        self.get_logger().info("Sent home command — all joints to 0 degrees")


def main():
    rclpy.init()
    node = JointCommandCLI()

    print_help()

    try:
        while rclpy.ok():
            try:
                raw = input("cmd> ").strip().lower()
            except EOFError:
                break

            if not raw:
                continue

            if raw in ("quit", "exit", "q"):
                print("Exiting.")
                break

            if raw in ("help", "h", "?"):
                print_help()
                continue

            if raw == "home":
                node.send_home()
                continue

            parts = raw.split()
            if len(parts) != 2:
                print(f"  Error: expected '<index> <degrees>', got '{raw}'")
                print("  Type 'help' for usage.")
                continue

            try:
                joint_index = int(parts[0])
                degrees = float(parts[1])
            except ValueError:
                print("  Error: joint index must be an integer and degrees must be a number.")
                continue

            if joint_index < 0 or joint_index >= NUM_JOINTS:
                print(f"  Error: joint index {joint_index} out of range. Valid range: 0-{NUM_JOINTS - 1}")
                continue

            node.send_command(joint_index, degrees)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()