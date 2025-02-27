# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

from typing import List, Tuple

import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

from synchros2.filters import Filter, TransformFilter


def test_transform_wait() -> None:
    source = Filter()
    tf_buffer = tf2_ros.Buffer()
    tf_filter = TransformFilter(source, "map", tf_buffer, tolerance_sec=1.0)
    sink: List[Tuple[PoseStamped, TransformStamped]] = []
    tf_filter.registerCallback(lambda *msgs: sink.append((msgs[0], msgs[1])))
    assert len(sink) == 0
    pose_message = PoseStamped()
    pose_message.header.frame_id = "odom"
    pose_message.header.stamp.sec = 1
    pose_message.pose.position.x = 1.0
    pose_message.pose.orientation.w = 1.0
    source.signalMessage(pose_message)
    assert len(sink) == 0
    transform_message = TransformStamped()
    transform_message.header.frame_id = "map"
    transform_message.child_frame_id = "odom"
    transform_message.transform.rotation.w = 1.0
    tf_buffer.set_transform_static(transform_message, "pytest")
    assert len(sink) == 1
    filtered_pose_message, filtered_transform_message = sink.pop()
    assert filtered_pose_message.header.frame_id == pose_message.header.frame_id
    assert filtered_pose_message.pose.position.x == pose_message.pose.position.x
    assert filtered_pose_message.pose.orientation.w == pose_message.pose.orientation.w
    assert filtered_transform_message.header.frame_id == transform_message.header.frame_id
    assert filtered_transform_message.child_frame_id == transform_message.child_frame_id
    assert filtered_transform_message.transform.rotation.w == transform_message.transform.rotation.w


def test_old_transform_filtering() -> None:
    source = Filter()
    tf_buffer = tf2_ros.Buffer()
    tf_filter = TransformFilter(source, "map", tf_buffer, tolerance_sec=2.0)
    sink: List[Tuple[PoseStamped, TransformStamped]] = []
    tf_filter.registerCallback(lambda *msgs: sink.append((msgs[0], msgs[1])))
    assert len(sink) == 0
    first_pose_message = PoseStamped()
    first_pose_message.header.frame_id = "odom"
    first_pose_message.pose.position.x = 1.0
    first_pose_message.pose.orientation.w = 1.0
    source.signalMessage(first_pose_message)
    assert len(sink) == 0
    second_pose_message = PoseStamped()
    second_pose_message.header.frame_id = "odom"
    second_pose_message.header.stamp.sec = 10
    second_pose_message.pose.position.x = 2.0
    second_pose_message.pose.orientation.w = 1.0
    source.signalMessage(second_pose_message)
    assert len(sink) == 0
    transform_message = TransformStamped()
    transform_message.header.frame_id = "map"
    transform_message.child_frame_id = "odom"
    transform_message.transform.rotation.w = 1.0
    tf_buffer.set_transform_static(transform_message, "pytest")
    assert len(sink) == 1
    filtered_pose_message, filtered_transform_message = sink.pop()
    assert filtered_pose_message.header.frame_id == second_pose_message.header.frame_id
    assert filtered_pose_message.pose.position.x == second_pose_message.pose.position.x
    assert filtered_pose_message.pose.orientation.w == second_pose_message.pose.orientation.w
    assert filtered_transform_message.header.frame_id == transform_message.header.frame_id
    assert filtered_transform_message.child_frame_id == transform_message.child_frame_id
    assert filtered_transform_message.transform.rotation.w == transform_message.transform.rotation.w
