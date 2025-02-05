#  Copyright (c) 2025 Boston Dynamics AI Institute Inc.  All rights reserved.

import std_msgs.msg
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from synchros2.futures import wait_for_future
from synchros2.publisher import Publisher
from synchros2.scope import ROSAwareScope

DEFAULT_QOS_PROFILE = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL,
    depth=1,
)


def test_publisher_matching_subscriptions(ros: ROSAwareScope) -> None:
    """Asserts that checking for subscription matching on a publisher works as expected."""
    assert ros.node is not None
    sequence = Publisher(  # type: ignore[var-annotated]
        std_msgs.msg.Int8,
        "sequence",
        qos_profile=DEFAULT_QOS_PROFILE,
        node=ros.node,
    )
    assert sequence.matched_subscriptions == 0
    future = sequence.subscription_matches(1)
    assert not future.done()
    future.cancel()

    ros.node.create_subscription(
        std_msgs.msg.Int8,
        "sequence",
        qos_profile=DEFAULT_QOS_PROFILE,
        callback=lambda msg: None,
    )
    assert wait_for_future(sequence.subscription_matches(1), timeout_sec=5.0)  # type: ignore[arg-type]
    assert sequence.matched_subscriptions == 1
