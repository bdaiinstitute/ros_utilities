# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

from geometry_msgs.msg import TransformStamped

from synchros2.scope import ROSAwareScope
from synchros2.static_transform_broadcaster import StaticTransformBroadcaster
from synchros2.tf_listener_wrapper import TFListenerWrapper


def test_static_tf_burst(ros: ROSAwareScope) -> None:
    """Asserts that all static transforms broadcasted in sequence are kept."""
    assert ros.node is not None

    tf_broadcaster = StaticTransformBroadcaster(ros.node)

    stamp = ros.node.get_clock().now().to_msg()

    world_to_fiducial_a_transform = TransformStamped()
    world_to_fiducial_a_transform.header.stamp = stamp
    world_to_fiducial_a_transform.header.frame_id = "world"
    world_to_fiducial_a_transform.child_frame_id = "fiducial_a"
    world_to_fiducial_a_transform.transform.rotation.w = 1.0
    tf_broadcaster.sendTransform(world_to_fiducial_a_transform)

    body_to_head_transform = TransformStamped()
    body_to_head_transform.header.stamp = stamp
    body_to_head_transform.header.frame_id = "body"
    body_to_head_transform.child_frame_id = "head"
    body_to_head_transform.transform.rotation.z = -1.0

    head_to_camera_transform = TransformStamped()
    head_to_camera_transform.header.stamp = stamp
    head_to_camera_transform.header.frame_id = "head"
    head_to_camera_transform.child_frame_id = "camera"
    head_to_camera_transform.transform.rotation.z = 1.0
    tf_broadcaster.sendTransform([body_to_head_transform, head_to_camera_transform])

    world_to_fiducial_a_transform = TransformStamped()
    world_to_fiducial_a_transform.header.stamp = stamp
    world_to_fiducial_a_transform.header.frame_id = "world"
    world_to_fiducial_a_transform.child_frame_id = "fiducial_a"
    world_to_fiducial_a_transform.transform.translation.x = 1.0
    world_to_fiducial_a_transform.transform.rotation.w = 1.0

    footprint_to_body_transform = TransformStamped()
    footprint_to_body_transform.header.stamp = stamp
    footprint_to_body_transform.header.frame_id = "footprint"
    footprint_to_body_transform.child_frame_id = "body"
    footprint_to_body_transform.transform.rotation.w = 1.0
    tf_broadcaster.sendTransform([world_to_fiducial_a_transform, footprint_to_body_transform])

    tf_listener = TFListenerWrapper(ros.node)
    transform = tf_listener.lookup_a_tform_b("footprint", "camera", timeout_sec=2.0, wait_for_frames=True)
    assert transform.transform.rotation.w == 1.0
    transform = tf_listener.lookup_a_tform_b("world", "fiducial_a", timeout_sec=2.0, wait_for_frames=True)
    assert transform.transform.translation.x == 1.0
