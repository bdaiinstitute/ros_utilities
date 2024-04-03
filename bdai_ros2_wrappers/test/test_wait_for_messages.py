# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
from typing import Any

import std_msgs.msg
from enum import EnumMeta
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from bdai_ros2_wrappers.node import Node
from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.subscription import wait_for_messages

def latch(depth: int = 1, durability: EnumMeta = QoSDurabilityPolicy.TRANSIENT_LOCAL) -> QoSProfile:
    return QoSProfile(depth=depth, durability=durability)

class NodeFoo(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("foo", **kwargs)
        self.pub = self.create_publisher(std_msgs.msg.String, "/test1", latch(depth=10))
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
        latched_topics={"/test1"}
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
        latched_topics={"/test1"}
    )
    assert messages == (None, None) or messages == (
        std_msgs.msg.String(data="hello from bar"),
        std_msgs.msg.String(data="hello from foo"),
    )
