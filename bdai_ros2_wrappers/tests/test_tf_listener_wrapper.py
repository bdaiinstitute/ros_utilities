# Copyright (c) 2023 Boston Dynamics AI Institute, Inc.  All rights reserved.
import time
import unittest
from threading import Thread
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from rclpy import Context
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import ExtrapolationException, LookupException, TransformBroadcaster

from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper

ROBOT = "test_robot"
CAMERA = "camera_1"
FRAME_ID = f"{ROBOT}/body"
CHILD_FRAME_ID = f"{ROBOT}/{CAMERA}"


def equal_transform(a: Transform, b: Transform) -> bool:
    return (
        a.translation.x == b.translation.x
        and a.translation.y == b.translation.y
        and a.translation.z == b.translation.z
        and a.rotation.w == b.rotation.w
        and a.rotation.x == b.rotation.x
        and a.rotation.y == b.rotation.y
        and a.rotation.z == a.rotation.z
    )


class MockTfPublisherNode(Node):
    def __init__(self, context: Context, frame_id: str, child_frame_id: str) -> None:
        super().__init__("mock_tf_publisher", context=context)

        self._frame_id = frame_id
        self._child_frame_id = child_frame_id
        self._tf_broadcaster = TransformBroadcaster(self)
        self._executor: Optional[SingleThreadedExecutor] = SingleThreadedExecutor(context=context)
        self._executor.add_node(self)
        self._thread: Optional[Thread] = Thread(target=self._spin)
        self._thread.start()

    def _spin(self) -> None:
        if self._executor is not None:
            try:
                self._executor.spin()
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

    def publish_transform(self, trans: Transform, timestamp: Optional[Time]) -> None:
        t = TransformStamped()

        if timestamp is not None:
            t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self._frame_id
        t.child_frame_id = self._child_frame_id

        t.transform = trans

        self._tf_broadcaster.sendTransform(t)

    def shutdown(self) -> None:
        if self._tf_broadcaster is not None:
            self._tf_broadcaster = None
        if self._executor is not None:
            self._executor.shutdown()
            self._executor.remove_node(self)
            self._executor = None
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self.destroy_node()


class TfListenerWrapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context: Optional[Context] = Context()
        rclpy.init(context=self.context)
        self.tf_publisher: Optional[MockTfPublisherNode] = MockTfPublisherNode(self.context, FRAME_ID, CHILD_FRAME_ID)
        self.tf_listener: Optional[TFListenerWrapper] = TFListenerWrapper(node_name="test", context=self.context)

    def tearDown(self) -> None:
        if self.tf_listener is not None:
            self.tf_listener.shutdown()
            self.tf_listener = None
        if self.tf_publisher is not None:
            self.tf_publisher.shutdown()
            self.tf_publisher = None
        rclpy.shutdown(context=self.context)
        self.context = None

    def test_non_existant_transform(self) -> None:
        if self.tf_listener is None or self.tf_publisher is None:
            self.fail()
        timestamp = self.tf_publisher.get_clock().now()
        self.assertRaises(LookupException, self.tf_listener.lookup_a_tform_b, FRAME_ID, CHILD_FRAME_ID, timestamp)

    def test_non_existant_transform_timeout(self) -> None:
        if self.tf_listener is None or self.tf_publisher is None:
            self.fail()
        timestamp = self.tf_publisher.get_clock().now()
        start = time.time()
        self.assertRaises(
            LookupException, self.tf_listener.lookup_a_tform_b, FRAME_ID, CHILD_FRAME_ID, timestamp, timeout=20.0
        )
        self.assertLess(time.time() - start, 10.0)

    def test_existing_transform(self) -> None:
        if self.tf_listener is None or self.tf_publisher is None:
            self.fail()
        timestamp = self.tf_publisher.get_clock().now()
        trans = Transform(translation=Vector3(x=1.0, y=2.0, z=3.0), rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0))
        self.tf_publisher.publish_transform(trans, timestamp)
        time.sleep(0.2)
        t = self.tf_listener.lookup_a_tform_b(FRAME_ID, CHILD_FRAME_ID, timestamp)
        self.assertTrue(equal_transform(t.transform, trans))

    def test_future_transform_extrapolation_exception(self) -> None:
        if self.tf_listener is None or self.tf_publisher is None:
            self.fail()
        timestamp = self.tf_publisher.get_clock().now()
        trans = Transform(translation=Vector3(x=1.0, y=2.0, z=3.0), rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0))
        self.tf_publisher.publish_transform(trans, timestamp)
        time.sleep(0.2)
        timestamp = self.tf_publisher.get_clock().now()
        self.assertRaises(
            ExtrapolationException, self.tf_listener.lookup_a_tform_b, FRAME_ID, CHILD_FRAME_ID, timestamp
        )

    def test_future_transform_insufficient_wait(self) -> None:
        if self.tf_listener is None or self.tf_publisher is None:
            self.fail()
        timestamp = self.tf_publisher.get_clock().now()
        trans = Transform(translation=Vector3(x=1.0, y=2.0, z=3.0), rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0))
        self.tf_publisher.publish_transform(trans, timestamp)

        delay = 2

        def _delayed_publish() -> None:
            if self.tf_publisher is None:
                self.fail()
            time.sleep(delay)
            delayed_timestamp = self.tf_publisher.get_clock().now()
            self.tf_publisher.publish_transform(trans, delayed_timestamp)

        thread = Thread(target=_delayed_publish)
        thread.start()

        time.sleep(0.2)
        timestamp = self.tf_publisher.get_clock().now() + Duration(seconds=delay)
        self.assertRaises(
            ExtrapolationException, self.tf_listener.lookup_a_tform_b, FRAME_ID, CHILD_FRAME_ID, timestamp, timeout=0.5
        )
        thread.join()

    def test_future_transform_wait(self) -> None:
        if self.tf_listener is None or self.tf_publisher is None:
            self.fail()
        timestamp = self.tf_publisher.get_clock().now()
        trans = Transform(translation=Vector3(x=1.0, y=2.0, z=3.0), rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0))
        self.tf_publisher.publish_transform(trans, timestamp)

        delay = 1

        def _delayed_publish() -> None:
            if self.tf_publisher is None:
                self.fail()
            time.sleep(delay + 0.001)
            delayed_timestamp = self.tf_publisher.get_clock().now()
            self.tf_publisher.publish_transform(trans, delayed_timestamp)

        thread = Thread(target=_delayed_publish)
        thread.start()

        timestamp += Duration(seconds=delay)
        t = self.tf_listener.lookup_a_tform_b(FRAME_ID, CHILD_FRAME_ID, timestamp, wait_for_frames=True, timeout=2)
        self.assertTrue(equal_transform(t.transform, trans))
        thread.join()


if __name__ == "__main__":
    unittest.main()
