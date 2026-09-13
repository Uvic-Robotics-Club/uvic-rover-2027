#!/usr/bin/env python3
"""
joint_cmd_cli.py - Interactive CLI for commanding rover arm joints.

Usage:
    ros2 run rover_arm joint_cmd_cli.py

Input format:
    <joint_index> <degrees>

Example:
    0 45        → rotate turret_joint to 45 degrees
    1 -30       → rotate shoulder_joint to -30 degrees
    home        → move all joints to home position (0 degrees)
    quit        → exit the CLI

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
import sys


# Joint index to name mapping — must match arm_hal.hpp JOINT_NAMES
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
    print("Input format: <joint_index> <degrees>")
    print("Commands:")
    print("  home      move all joints to 0 degrees")
    print("  help      show this message")
    print("  quit      exit")
    print("\nJoint index map:")
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {i} = {name}")
    print()


class JointCommandCLI(Node):

    def __init__(self):
        super().__init__('joint_cmd_cli')

        self.publisher = self.create_publisher(
            JointState,
            '/arm/cmd_joint',
            10
        )

        self.get_logger().info("Joint command CLI ready — publishing to /arm/cmd_joint")

    def send_command(self, joint_index: int, degrees: float):
        """Publish a single joint command."""
        radians = math.radians(degrees)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [JOINT_NAMES[joint_index]]
        msg.position = [radians]

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Commanded {JOINT_NAMES[joint_index]} to {degrees:.1f} deg ({radians:.4f} rad)"
        )

    def send_home(self):
        """Publish home command — all joints to 0."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [0.0] * NUM_JOINTS

        self.publisher.publish(msg)
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
                # Handle piped input ending
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

            # Parse "index degrees"
            parts = raw.split()
            if len(parts) != 2:
                print(f"  Error: expected '<index> <degrees>', got '{raw}'")
                print("  Type 'help' for usage.")
                continue

            try:
                joint_index = int(parts[0])
                degrees = float(parts[1])
            except ValueError:
                print(f"  Error: joint index must be an integer and degrees must be a number.")
                continue

            if joint_index < 0 or joint_index >= NUM_JOINTS:
                print(f"  Error: joint index {joint_index} out of range. Valid range: 0-{NUM_JOINTS - 1}")
                continue

            node.send_command(joint_index, degrees)

            # Spin once to process any callbacks
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()