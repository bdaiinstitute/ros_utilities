# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import time

from std_msgs.msg import Int8

from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.subscription import wait_for_message


def test_wait_for_message(ros: ROSAwareScope) -> None:
    """Asserts that wait for message works as expected."""
    assert ros.node is not None
    pub = ros.node.create_publisher(Int8, "test", 1)
    assert not wait_for_message(Int8, "test", node=ros.node, timeout_sec=0.5)

    def deferred_publish() -> None:
        time.sleep(1.0)
        pub.publish(Int8(data=1))

    assert ros.executor is not None
    ros.executor.create_task(deferred_publish)
    message = wait_for_message(Int8, "test", node=ros.node, timeout_sec=10.0)
    assert message is not None
    assert message.data == 1
