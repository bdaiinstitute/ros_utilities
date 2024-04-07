# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
from enum import EnumMeta
from typing import Any

import std_msgs.msg
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from bdai_ros2_wrappers.node import Node
from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.subscription import wait_for_messages


TRANSIENT_LOCAL = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

class NodeFoo(Node):

    def __init__(self, **kwargs: Any) -> None:
        super().__init__("foo", **kwargs)
        self.pub = self.create_publisher(std_msgs.msg.String, "/test1", TRANSIENT_LOCAL)
        self.pub.publish(std_msgs.msg.String(data="hello from foo"))


class NodeBar(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("bar", **kwargs)
        self.pub = self.create_publisher(std_msgs.msg.String, "/test2", 10)
        self.timer = self.create_timer(0.5, self.publish_msg)

    def publish_msg(self) -> None:
        self.pub.publish(std_msgs.msg.String(data="hello from bar"))


def test_wait_for_messages(ros: ROSAwareScope) -> None:
    ros.load(NodeFoo)
    ros.load(NodeBar)

    node_wfm = ros.node

    messages = wait_for_messages(
        node_wfm,
        ["/test1", "/test2"],
        [std_msgs.msg.String, std_msgs.msg.String],
        allow_headerless=True,
        verbose=True,
        delay=0.5,
        timeout=20,
        qos_profiles={"/test1": TRANSIENT_LOCAL},
    )
    assert messages == (None, None) or messages == (
        std_msgs.msg.String(data="hello from foo"),
        std_msgs.msg.String(data="hello from bar"),
    )

    messages = wait_for_messages(
        node_wfm,
        ["/test2", "/test1"],
        [std_msgs.msg.String, std_msgs.msg.String],
        allow_headerless=True,
        verbose=True,
        delay=0.5,
        timeout=20,
        qos_profiles={"/test1": TRANSIENT_LOCAL}
    )
    assert messages == (None, None) or messages == (
        std_msgs.msg.String(data="hello from bar"),
        std_msgs.msg.String(data="hello from foo"),
    )
