# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.
import itertools
import time

from typing import Any, Iterator, cast

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Int8

from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.node import Node
from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.subscription import Subscription, wait_for_message, wait_for_messages

DEFAULT_QOS_PROFILE = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL,
    depth=1,
)


def test_wait_for_message(ros: ROSAwareScope) -> None:
    """Asserts that waiting for a message works as expected."""
    assert ros.node is not None
    pub = ros.node.create_publisher(Int8, "test", DEFAULT_QOS_PROFILE)
    assert not wait_for_message(Int8, "test", node=ros.node, timeout_sec=0.5)

    def deferred_publish() -> None:
        time.sleep(0.5)
        pub.publish(Int8(data=1))

    assert ros.executor is not None
    ros.executor.create_task(deferred_publish)
    message = wait_for_message(Int8, "test", node=ros.node, timeout_sec=5.0)
    assert message is not None
    assert message.data == 1


def test_subscription_future_wait(ros: ROSAwareScope) -> None:
    """Asserts that waiting for a subscription update works as expected."""
    assert ros.node is not None
    pub = ros.node.create_publisher(Int8, "sequence", DEFAULT_QOS_PROFILE)
    sequence = Subscription(Int8, "sequence", DEFAULT_QOS_PROFILE, node=ros.node)

    pub.publish(Int8(data=1))

    assert wait_for_future(sequence.update, timeout_sec=5.0)
    assert cast(Int8, sequence.latest).data == 1


def test_subscription_iteration(ros: ROSAwareScope) -> None:
    """Asserts that iterating over subscription messages works as expected."""
    assert ros.node is not None

    pub = ros.node.create_publisher(
        Int8,
        "sequence",
        DEFAULT_QOS_PROFILE,
    )
    sequence = Subscription(
        Int8,
        "sequence",
        DEFAULT_QOS_PROFILE,
        history_length=3,
        node=ros.node,
    )

    expected_sequence_numbers = [1, 10, 100]

    def deferred_publish() -> None:
        time.sleep(0.5)
        for num in expected_sequence_numbers:
            pub.publish(Int8(data=num))

    assert ros.executor is not None
    ros.executor.create_task(deferred_publish)
    message_stream: Iterator[Int8] = sequence.stream(timeout_sec=5.0)

    streamed_numbers = [msg.data for msg in itertools.islice(message_stream, 3)]
    assert expected_sequence_numbers == streamed_numbers

    historic_numbers = [msg.data for msg in sequence.history]
    assert expected_sequence_numbers == historic_numbers


def test_subscription_cancelation(ros: ROSAwareScope) -> None:
    """Asserts that cancelling a subscription works as expected."""
    assert ros.node is not None
    pub = ros.node.create_publisher(Int8, "sequence", DEFAULT_QOS_PROFILE)
    sequence = Subscription(Int8, "sequence", DEFAULT_QOS_PROFILE, node=ros.node)

    pub.publish(Int8(data=1))

    assert wait_for_future(sequence.update, timeout_sec=5.0)

    def deferred_cancellation() -> None:
        time.sleep(0.5)
        sequence.unsubscribe()

    assert ros.executor is not None
    ros.executor.create_task(deferred_cancellation)

    streamed_numbers = [msg.data for msg in sequence.stream()]
    assert len(streamed_numbers) == 1
    assert streamed_numbers[0] == 1

    pub.publish(Int8(data=10))

    assert not wait_for_future(sequence.update, timeout_sec=5.0)
    assert sequence.update.cancelled()

    historic_numbers = [msg.data for msg in sequence.history]
    assert len(historic_numbers) == 1
    assert historic_numbers[0] == 1

    assert cast(Int8, sequence.latest).data == 1


def test_wait_for_messages(ros: ROSAwareScope) -> None:
    class NodeFoo(Node):
        def __init__(self, **kwargs: Any) -> None:
            super().__init__("foo", **kwargs)
            self.pub = self.create_publisher(std_msgs.msg.String, "/test1", 10)
            self.timer = self.create_timer(0.5, self.publish_msg)

        def publish_msg(self) -> None:
            self.pub.publish(std_msgs.msg.String(data="hello from foo"))

    class NodeBar(Node):
        def __init__(self, **kwargs: Any) -> None:
            super().__init__("bar", **kwargs)
            self.pub = self.create_publisher(std_msgs.msg.String, "/test2", 10)
            self.timer = self.create_timer(0.5, self.publish_msg)

        def publish_msg(self) -> None:
            self.pub.publish(std_msgs.msg.String(data="hello from bar"))

    ros.load(NodeFoo)
    ros.load(NodeBar)

    node_wfm = ros.node

    messages = wait_for_messages(
        ["/test1", "/test2"],
        [std_msgs.msg.String, std_msgs.msg.String],
        allow_headerless=True,
        delay=0.5,
        timeout_sec=20,
        node=node_wfm,
    )
    assert messages is None or messages == (
        std_msgs.msg.String(data="hello from foo"),
        std_msgs.msg.String(data="hello from bar"),
    )

    messages = wait_for_messages(
        ["/test2", "/test1"],
        [std_msgs.msg.String, std_msgs.msg.String],
        allow_headerless=True,
        delay=0.5,
        timeout_sec=20,
        node=node_wfm,
    )
    assert messages is None or messages == (
        std_msgs.msg.String(data="hello from bar"),
        std_msgs.msg.String(data="hello from foo"),
    )

    # Test the case where the topic doesn't exist - timeout expected
    messages = wait_for_messages(
        ["/test3", "/test1"],
        [std_msgs.msg.String, std_msgs.msg.String],
        allow_headerless=True,
        timeout_sec=1,
        node=node_wfm,
    )
    assert messages is None
