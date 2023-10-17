# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import typing

import rclpy.node
import rclpy.qos
import rclpy.task

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.futures import wait_for_future

MessageT = typing.TypeVar("MessageT")


def wait_for_message_async(
    msg_type: MessageT,
    topic_name: str,
    *,
    qos_profile: typing.Union[rclpy.qos.QoSProfile, int] = 1,
    node: typing.Optional[rclpy.node.Node] = None,
) -> rclpy.task.Future:
    """
    Wait for message on a given topic asynchronously.

    Args:
        msg_type: type of message to wait for.
        topic_name: name of the topic to wait on.
        qos_profile: optional QoS profile for temporary topic subscription.
        node: optional node for temporary topic subscription, defaults to
        the current process-wide node (if any).

    Returns:
        A future for the incoming message.

    Raises:
        RuntimeError: if no node is available.
    """
    node = node or scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    future = rclpy.task.Future()

    def callback(msg: MessageT) -> None:
        if not future.done():
            future.set_result(msg)

    sub = node.create_subscription(msg_type, topic_name, callback, qos_profile)
    future.add_done_callback(lambda future: node.destroy_subscription(sub))
    return future


def wait_for_message(
    msg_type: MessageT,
    topic_name: str,
    timeout_sec: typing.Optional[float] = None,
    *,
    node: typing.Optional[rclpy.node.Node] = None,
    **kwargs: typing.Any,
) -> typing.Optional[MessageT]:
    """
    Wait for message on a given topic synchronously.

    Args:
        msg_type: type of message to wait for.
        topic_name: name of the topic to wait on.
        timeout_sec: optional timeout, in seconds, for the wait.

    See `wait_for_message_async` documentation for a reference on
    additional keyword arguments.

    Returns:
        The message received, or None on timeout.
    """
    node = node or scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    future = wait_for_message_async(msg_type, topic_name, node=node, **kwargs)
    if not wait_for_future(future, timeout_sec, context=node.context):
        future.cancel()
        return None
    return future.result()
