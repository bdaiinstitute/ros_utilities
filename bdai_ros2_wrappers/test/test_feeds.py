# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.


import tf2_ros
from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
    TwistStamped,
)
from message_filters import SimpleFilter

from bdai_ros2_wrappers.feeds import (
    AdaptedMessageFeed,
    FramedMessageFeed,
    MessageFeed,
    SynchronizedMessageFeed,
)
from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.utilities import ensure


def test_framed_message_feed(ros: ROSAwareScope) -> None:
    tf_buffer = tf2_ros.Buffer()
    pose_message_feed = MessageFeed(SimpleFilter())
    framed_message_feed = FramedMessageFeed(
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
    pose_message_feed = MessageFeed(SimpleFilter())
    twist_message_feed = MessageFeed(SimpleFilter())
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

    pose_message, twist_message = ensure(synchronized_message_feed.latest)
    assert pose_message.pose.position.x == expected_pose_message.pose.position.x
    assert twist_message.twist.linear.x == expected_twist_message.twist.linear.x


def test_adapted_message_feed(ros: ROSAwareScope) -> None:
    pose_message_feed = MessageFeed(SimpleFilter())
    position_message_feed = AdaptedMessageFeed(
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

    position_message = ensure(position_message_feed.latest)
    # no copies are expected, thus an identity check is valid
    assert position_message is expected_pose_message.pose.position
