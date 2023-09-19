# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import os
import unittest
from typing import Optional

import rclpy
from rclpy import Context
from rclpy.node import Node
from std_msgs.msg import String

from bdai_ros2_wrappers.rosbag_recorder import ROSBagRecorder

BAG_FILE_DIR = os.path.join(os.getenv("BDAI", "/workspaces/bdai"), "recording")
BAG_FILE_NAME = "unit_test_ros2_bag"


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


class ROSBagRecorderTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context: Optional[Context] = Context()
        rclpy.init(context=self.context)
        self.rosbag_recorder = ROSBagRecorder(os.path.join(BAG_FILE_NAME, BAG_FILE_DIR))
        self.minimal_pub = MinimalPublisher()

        self.rosbag_recorder.start()
        rclpy.spin(self.minimal_pub)

    def tearDown(self) -> None:
        self.rosbag_recorder.stop()
        self.minimal_pub.destroy_node()
        rclpy.shutdown(context=self.context)
        self.context = None

    def test_bag_creation(self) -> None:
        """
        Tests normal operation of recording a minimal publisher, ensure rosbag is generated
        """
        expected_bag_file_path = os.path.join(BAG_FILE_DIR, BAG_FILE_NAME)
        self.assertEqual(expected_bag_file_path, self.rosbag_recorder.bag_path)
        self.assertTrue(os.path.exists(self.rosbag_recorder.bag_path))


if __name__ == "__main__":
    unittest.main()
