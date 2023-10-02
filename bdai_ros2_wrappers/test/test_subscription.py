# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import time
import typing

import pytest
import rclpy
from std_msgs.msg import Int8

from bdai_ros2_wrappers.process import ROSAwareScope
from bdai_ros2_wrappers.subscription import wait_for_message


@pytest.fixture
def ros() -> typing.Iterable[ROSAwareScope]:
    rclpy.init()
    try:
        with ROSAwareScope("fixture") as scope:
            yield scope
    finally:
        rclpy.try_shutdown()


def test_wait_for_message(ros: ROSAwareScope) -> None:
    """Asserts that wait for message works as expected."""
    pub = ros.node.create_publisher(Int8, "test", 1)
    assert not wait_for_message(Int8, "test", node=ros.node, timeout_sec=0.5)

    def deferred_publish() -> None:
        time.sleep(1.0)
        pub.publish(Int8(data=1))

    ros.executor.create_task(deferred_publish)
    message = wait_for_message(Int8, "test", node=ros.node, timeout_sec=10.0)
    assert message is not None
    assert message.data == 1
