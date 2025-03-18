#  Copyright (c) 2025 Boston Dynamics AI Institute LLC.  All rights reserved.
import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Int8

from synchros2.graph import ensure_num_publishers, ensure_num_subscriptions
from synchros2.scope import ROSAwareScope

DEFAULT_QOS_PROFILE = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL,
    depth=1,
)


def test_ensure_num_publishers(ros: ROSAwareScope) -> None:
    """Asserts that checking for publisher matching on a subscription works as expected."""
    assert ros.node is not None
    assert ensure_num_publishers(ros.node, "sequence", 0, timeout_sec=5.0) == 0
    with pytest.raises(ValueError):
        ensure_num_publishers(ros.node, "sequence", 1, timeout_sec=1.0)
    ros.node.create_publisher(Int8, "sequence", DEFAULT_QOS_PROFILE)
    assert ensure_num_publishers(ros.node, "sequence", 1, timeout_sec=5.0) == 1


def test_ensure_num_subscriptions(ros: ROSAwareScope) -> None:
    """Asserts that checking for publisher matching on a subscription works as expected."""
    assert ros.node is not None
    assert ensure_num_subscriptions(ros.node, "sequence", 0, timeout_sec=5.0) == 0
    with pytest.raises(ValueError):
        ensure_num_subscriptions(ros.node, "sequence", 1, timeout_sec=1.0)
    ros.node.create_subscription(Int8, "sequence", lambda _: None, DEFAULT_QOS_PROFILE)
    assert ensure_num_subscriptions(ros.node, "sequence", 1, timeout_sec=5.0) == 1
