# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import unittest
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bdai_ros2_wrappers.rosbag_recorder import ROSBagRecorder

BAG_FILE_DIR = Path.home() / "recordings"
BAG_FILE_NAME = "unit_test_ros2_bag"
PUBLISH_COUNT = 20


class MinimalPublisher(Node):
    def __init__(self) -> None:
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self) -> None:
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        if self.i <= PUBLISH_COUNT:
            self.get_logger().info("Publish Count reached. Exiting test.")
            raise SystemExit


class ROSBagRecorderTest(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()

        self.rosbag_recorder = ROSBagRecorder(Path(BAG_FILE_DIR, BAG_FILE_NAME))
        self.minimal_pub = MinimalPublisher()

        try:
            self.rosbag_recorder.start()
            rclpy.spin(self.minimal_pub)
        except SystemExit:
            pass

    def tearDown(self) -> None:
        self.rosbag_recorder.stop()
        self.minimal_pub.destroy_node()
        rclpy.shutdown()

    def test_bag_creation(self) -> None:
        """
        Tests normal operation of recording a minimal publisher, ensure rosbag is generated
        """
        expected_bag_file_path = Path(BAG_FILE_DIR, BAG_FILE_NAME)
        self.assertEqual(expected_bag_file_path, self.rosbag_recorder.bag_path)
        self.assertTrue(self.rosbag_recorder.bag_path.exists())


if __name__ == "__main__":
    unittest.main()
