
#!/usr/bin/env python3
"""Check ZED 2i RGB/depth/point-cloud message synchronization."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


TOPICS = {
    "rgb": (Image, "/zed/zed_node/rgb/color/rect/image"),
    "depth": (Image, "/zed/zed_node/depth/depth_registered"),
    "cloud": (PointCloud2, "/zed/zed_node/point_cloud/cloud_registered"),
}


class ZedSyncTest(Node):
    def __init__(self, include_cloud=False):
        super().__init__("zed_sync_test")

        self.names = ["rgb", "depth", "cloud"] if include_cloud else ["rgb", "depth"]
        self.timestamps = {name: set() for name in self.names}
        self.counts = {name: 0 for name in self.names}
        self.start = time.monotonic()
        self.zed_subscriptions = []

        for name in self.names:
            msg_type, topic = TOPICS[name]

            sub = self.create_subscription(
                msg_type, topic,
                lambda msg, n=name: self.on_message(n, msg),
                qos_profile_sensor_data,
            )
            self.zed_subscriptions.append(sub)

        self.get_logger().info("Monitoring: " + ", ".join(self.names))

    def on_message(self, name, msg):
        stamp = msg.header.stamp
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec

        self.timestamps[name].add(timestamp_ns)
        self.counts[name] += 1

    def report(self):
        duration = time.monotonic() - self.start

        print("\n=== ZED 2i Synchronization Test ===")
        print(f"Duration: {duration:.1f} seconds")

        for name in self.names:
            count = self.counts[name]
            rate = count / duration if duration else 0.0

            print(f"{name.upper():6} messages={count:5} observed_rate={rate:.2f} Hz")

        matched = set.intersection(*(self.timestamps[name] for name in self.names))
        print(f"\nExact timestamp matches: {len(matched)}")

        if all(self.counts.values()):
            expected = min(len(self.timestamps[name]) for name in self.names)
            ratio = len(matched) / expected if expected else 0

            print(f"Match ratio: {ratio:.1%}")

            if ratio >= 0.95:
                print("PASS: Timestamp synchronization")
            else:
                print("FAIL: Insufficient timestamp matches")
        else:
            print("FAIL: One or more topics have no messages")

        print(
            "\nNote: Observed subscriber rates do not "
            "measure the camera's internal capture FPS."
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30, help="Test duration in seconds")
    parser.add_argument("--pointcloud", action="store_true", help="Include point-cloud timestamp matching")
    args = parser.parse_args()

    rclpy.init()
    node = ZedSyncTest(include_cloud=args.pointcloud)

    try:
        deadline = time.monotonic() + args.duration

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.report()
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
