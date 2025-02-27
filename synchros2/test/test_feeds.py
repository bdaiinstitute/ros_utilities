# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

from typing import Optional, Tuple, cast

import tf2_ros
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    TransformStamped,
    TwistStamped,
)

from synchros2.feeds import (
    AdaptedMessageFeed,
    ExactSynchronizedMessageFeed,
    FramedMessageFeed,
    MessageFeed,
    SynchronizedMessageFeed,
)
from synchros2.filters import Filter
from synchros2.scope import ROSAwareScope
from synchros2.utilities import ensure


def test_framed_message_feed(ros: ROSAwareScope) -> None:
    tf_buffer = tf2_ros.Buffer()
    pose_message_feed = MessageFeed[PoseStamped](Filter())
    framed_message_feed = FramedMessageFeed[Tuple[PoseStamped, TransformStamped]](
        pose_message_feed,
        target_frame_id="map",
        tf_buffer=tf_buffer,
        node=ros.node,
    )

    expected_transform_message = TransformStamped()
    expected_transform_message.header.frame_id = "map"
    expected_transform_message.child_frame_id = "odom"
    expected_transform_message.transform.translation.y = 1.0
    expected_transform_message.transform.rotation.w = 1.0
    tf_buffer.set_transform_static(expected_transform_message, "pytest")

    expected_pose_message = PoseStamped()
    expected_pose_message.header.frame_id = "odom"
    expected_pose_message.header.stamp.sec = 1
    expected_pose_message.pose.position.x = 1.0
    expected_pose_message.pose.orientation.w = 1.0
    pose_message_feed.link.signalMessage(expected_pose_message)

    pose_message, transform_message = ensure(framed_message_feed.latest)
    assert pose_message.pose.position.x == expected_pose_message.pose.position.x
    assert transform_message.transform.translation.y == expected_transform_message.transform.translation.y


def test_synchronized_message_feed(ros: ROSAwareScope) -> None:
    pose_message_feed = MessageFeed[PoseStamped](Filter())
    twist_message_feed = MessageFeed[TwistStamped](Filter())
    synchronized_message_feed = SynchronizedMessageFeed(
        pose_message_feed,
        twist_message_feed,
        node=ros.node,
    )

    expected_pose_message = PoseStamped()
    expected_pose_message.header.frame_id = "odom"
    expected_pose_message.header.stamp.sec = 1
    expected_pose_message.pose.position.x = 1.0
    expected_pose_message.pose.orientation.w = 1.0
    pose_message_feed.link.signalMessage(expected_pose_message)

    expected_twist_message = TwistStamped()
    expected_twist_message.header.frame_id = "base_link"
    expected_twist_message.header.stamp.sec = 1
    expected_twist_message.twist.linear.x = 1.0
    expected_twist_message.twist.angular.z = 1.0
    twist_message_feed.link.signalMessage(expected_twist_message)

    pose_message, twist_message = cast(
        Tuple[PoseStamped, TwistStamped],
        ensure(synchronized_message_feed.latest),
    )
    assert pose_message.pose.position.x == expected_pose_message.pose.position.x
    assert twist_message.twist.linear.x == expected_twist_message.twist.linear.x


def test_exact_synchronized_message_feed(ros: ROSAwareScope) -> None:
    pose_message_feed = MessageFeed[PoseStamped](Filter())
    twist_message_feed = MessageFeed[TwistStamped](Filter())
    synchronized_message_feed = ExactSynchronizedMessageFeed(
        pose_message_feed,
        twist_message_feed,
        node=ros.node,
    )

    expected_pose_message = PoseStamped()
    expected_pose_message.header.frame_id = "odom"
    expected_pose_message.header.stamp.sec = 1
    expected_pose_message.pose.position.x = 1.0
    expected_pose_message.pose.orientation.w = 1.0
    pose_message_feed.link.signalMessage(expected_pose_message)

    expected_twist_message = TwistStamped()
    expected_twist_message.header.frame_id = "base_link"
    expected_twist_message.header.stamp.sec = 1
    expected_twist_message.twist.linear.x = 1.0
    expected_twist_message.twist.angular.z = 1.0
    twist_message_feed.link.signalMessage(expected_twist_message)

    pose_message, twist_message = cast(
        Tuple[PoseStamped, TwistStamped],
        ensure(synchronized_message_feed.latest),
    )
    assert pose_message.pose.position.x == expected_pose_message.pose.position.x
    assert twist_message.twist.linear.x == expected_twist_message.twist.linear.x


def test_adapted_message_feed(ros: ROSAwareScope) -> None:
    pose_message_feed = MessageFeed[PoseStamped](Filter())
    position_message_feed = AdaptedMessageFeed[Point](
        pose_message_feed,
        fn=lambda message: message.pose.position,
    )

    expected_pose_message = PoseStamped()
    expected_pose_message.header.frame_id = "odom"
    expected_pose_message.header.stamp.sec = 1
    expected_pose_message.pose.position.x = 1.0
    expected_pose_message.pose.position.z = -1.0
    expected_pose_message.pose.orientation.w = 1.0
    pose_message_feed.link.signalMessage(expected_pose_message)

    position_message: Point = ensure(position_message_feed.latest)
    # no copies are expected, thus an identity check is valid
    assert position_message is expected_pose_message.pose.position


def test_masked_message_feed(ros: ROSAwareScope) -> None:
    pose_message_feed = MessageFeed[PoseStamped](Filter())
    position_masking_feed = AdaptedMessageFeed[Point](
        pose_message_feed,
        fn=lambda message: message if message.pose.position.x > 0.0 else None,
    )
    expected_pose_message0 = PoseStamped()
    expected_pose_message0.header.frame_id = "odom"
    expected_pose_message0.header.stamp.sec = 1
    expected_pose_message0.pose.position.x = -1.0
    expected_pose_message0.pose.position.z = -1.0
    expected_pose_message0.pose.orientation.w = 1.0
    pose_message_feed.link.signalMessage(expected_pose_message0)
    assert position_masking_feed.latest is None

    expected_pose_message1 = PoseStamped()
    expected_pose_message1.header.frame_id = "odom"
    expected_pose_message1.header.stamp.sec = 2
    expected_pose_message1.pose.position.x = 1.0
    expected_pose_message1.pose.position.z = -1.0
    expected_pose_message1.pose.orientation.w = 1.0
    pose_message_feed.link.signalMessage(expected_pose_message1)

    pose_message: Point = ensure(position_masking_feed.latest)
    # no copies are expected, thus an identity check is valid
    assert pose_message is expected_pose_message1


def test_message_feed_recalls(ros: ROSAwareScope) -> None:
    pose_message_feed = MessageFeed[PoseStamped](Filter())

    latest_message: Optional[PoseStamped] = None

    def callback(message: PoseStamped) -> None:
        nonlocal latest_message
        latest_message = message

    conn = pose_message_feed.recall(callback)

    first_pose_message = PoseStamped()
    first_pose_message.header.stamp.sec = 1
    pose_message_feed.link.signalMessage(first_pose_message)

    assert latest_message is not None
    assert latest_message.header.stamp.sec == first_pose_message.header.stamp.sec

    conn.close()

    second_pose_message = PoseStamped()
    second_pose_message.header.stamp.sec = 2
    pose_message_feed.link.signalMessage(second_pose_message)

    assert latest_message.header.stamp.sec != second_pose_message.header.stamp.sec
    assert latest_message.header.stamp.sec == first_pose_message.header.stamp.sec
