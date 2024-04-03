# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

from collections.abc import Sequence
from typing import Any, Iterator, Optional, TypeVar, Union

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.task import Future

import message_filters
import rclpy.callback_groups
import rclpy.impl
import rclpy.node
import rclpy.qos
import rclpy.task

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.utilities import Tape

MessageT = TypeVar("MessageT")


class Subscription:
    """An ergonomic interface to for topic subscription in ROS 2.

    Subscription instances wrap `rclpy.Subscription` instances and allow
    synchronous and asynchronous iteration and fetching of published data.
    """

    def __init__(
        self,
        message_type: MessageT,
        topic_name: str,
        qos_profile: Optional[Union[QoSProfile, int]] = None,
        history_length: Optional[int] = None,
        node: Optional[Node] = None,
        **kwargs: Any,
    ) -> None:
        """Initializes the subscription.

        Args:
            message_type: Target message type class (as generated by ``rosidl``).
            topic_name: Name of the target topic in the ROS 2 graph.
            qos_profile: an optional quality-of-service profile or simply a history depth
            to use for the underlying native subscription, defaults to a history depth of 1.
            history_length: optional historic data size, defaults to 1
            node: optional node for the underlying native subscription, defaults to
            the current process node.
            kwargs: other keyword arguments are used to create the underlying native subscription.
            See `rclpy.node.Node.create_subscription` documentation for further reference.
        """
        node = node or scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._node = node
        if history_length is None:
            history_length = 1
        if qos_profile is None:
            qos_profile = 1
        self._message_type = message_type
        self._topic_name = topic_name
        self._message_tape = Tape(history_length)
        self._topic_subscription = self._node.create_subscription(
            message_type,
            topic_name,
            self._message_tape.write,
            qos_profile,
            **kwargs,
        )
        self._node.context.on_shutdown(self._message_tape.close)

    @property
    def history(self) -> Sequence[Any]:
        """Gets the entire history of messages received so far."""
        return list(self._message_tape.content())

    @property
    def latest(self) -> Optional[Any]:
        """Gets the latest message received, if any."""
        return next(self._message_tape.content(), None)

    @property
    def update(self) -> Future:
        """Gets the a future to the next message yet to be received."""
        return self._message_tape.future_write

    def stream(
        self,
        *,
        forward_only: bool = False,
        buffer_size: Optional[int] = None,
        timeout_sec: Optional[float] = None,
    ) -> Iterator[Any]:
        """Iterates over messages as they come.

        Iteration stops when the given timeout expires or when the associated context
        is shutdown. Note that iterating over the message stream is a blocking operation.

        Args:
            forward_only: whether to ignore previosuly received messages.
            buffer_size: optional maximum size for the incoming messages buffer.
            If none is provided, the buffer will be grow unbounded.
            timeout_sec: optional timeout, in seconds, for a new message to be received.

        Returns:
            a lazy iterator over messages.
        """
        return self._message_tape.content(
            follow=True,
            forward_only=forward_only,
            buffer_size=buffer_size,
            timeout_sec=timeout_sec,
        )

    def cancel(self) -> None:
        """Cancels the message subscription if not cancelled already."""
        self._node.destroy_subscription(self._topic_subscription)
        self._message_tape.close()

    # Alias for improved readability
    unsubscribe = cancel


def wait_for_message_async(
    msg_type: MessageT,
    topic_name: str,
    *,
    qos_profile: Union[QoSProfile, int] = 1,
    node: Optional[Node] = None,
) -> Future:
    """Wait for message on a given topic asynchronously.

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
    future = Future()

    def callback(msg: MessageT) -> None:
        if not future.done():
            future.set_result(msg)

    sub = node.create_subscription(msg_type, topic_name, callback, qos_profile)
    future.add_done_callback(lambda future: node.destroy_subscription(sub))
    return future


def wait_for_message(
    msg_type: MessageT,
    topic_name: str,
    timeout_sec: Optional[float] = None,
    *,
    node: Optional[Node] = None,
    **kwargs: Any,
) -> Optional[MessageT]:
    """Wait for message on a given topic synchronously.

    Args:
        msg_type: type of message to wait for.
        topic_name: name of the topic to wait on.
        timeout_sec: optional timeout, in seconds, for the wait.
        node: An optional Node to provide. If none is provided, the one from the scope is used.
        kwargs: keyword argument to pass to `wait_for_message_async`

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


def wait_for_messages(
    node: rclpy.node.Node, topics: typing.List, mtypes: typing.List, **kwargs: typing.Any
) -> typing.Any:
    """Waits for messages to arrive at multiple topics within a given
    time window. Uses message_filters.ApproximateTimeSynchronizer.
    This function blocks until receiving the messages or when a given
    timeout expires. Assumes the given node is spinning by
    some external executor.

    Requires the user to pass in a node, since
    message_filters.Subscriber requires a node upon construction.

    Args:
        node (Node): the node being attached
        topics (list) List of topics
        mtypes (list) List of message types, one for each topic.
        delay  (float) The delay in seconds for which the messages
            could be synchronized.
        allow_headerless (bool): Whether it's ok for there to be
            no header in the messages.
        sleep (float) the amount of time to wait before checking
            whether messages are received
        timeout (float or None): Time in seconds to wait. None if forever.
            If exceeded timeout, self.messages will contain None for
            each topic.
        latched_topics (set): a set of topics for which the publisher latches (i.e.
            sets QoS durability to transient_local).
    """
    return _WaitForMessages(node, topics, mtypes, **kwargs).messages


class _WaitForMessages:
    def __init__(
        self,
        node: rclpy.node.Node,
        topics: typing.List,
        mtypes: typing.List,
        queue_size: int = 10,
        delay: float = 0.2,
        allow_headerless: bool = False,
        sleep: float = 0.5,
        timeout: typing.Optional[float] = None,
        verbose: bool = False,
        exception_on_timeout: bool = False,
        latched_topics: typing.Optional[typing.Set] = None,
        callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None,
    ) -> None:
        self.node = node
        self.messages: typing.Optional[typing.Tuple] = None
        self.verbose = verbose
        self.topics = topics
        self.timeout = timeout
        self.exception_on_timeout = exception_on_timeout
        self.has_timed_out = False
        if latched_topics is None:
            latched_topics = set()
        self.latched_topics = latched_topics
        self.logger = rclpy.impl.rcutils_logger.RcutilsLogger(name="wait_for_messages")

        if self.verbose:
            self.logger.info("initializing message filter ApproximateTimeSynchronizer")
        self.subs: typing.Optional[typing.List] = [
            self._message_filters_subscriber(mtype, topic, callback_group=callback_group)
            for topic, mtype in zip(topics, mtypes, strict=True)
        ]
        self.ts = message_filters.ApproximateTimeSynchronizer(
            self.subs,
            queue_size,
            delay,
            allow_headerless=allow_headerless,
        )
        self.ts.registerCallback(self._cb)

        try:
            self._start_time = self.node.get_clock().now()
            rate = self.node.create_rate(1.0 / sleep)
            while self.messages is None and not self.has_timed_out:
                if self.check_messages_received():
                    break
                rate.sleep()
        finally:
            self.destroy_subs()

    def _message_filters_subscriber(
        self,
        mtype: typing.Any,
        topic: str,
        callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None,
    ) -> message_filters.Subscriber:
        if topic in self.latched_topics:
            return message_filters.Subscriber(
                self.node,
                mtype,
                topic,
                qos_profile=rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL),
                callback_group=callback_group,
            )
        else:
            return message_filters.Subscriber(self.node, mtype, topic, callback_group=callback_group)

    def check_messages_received(self) -> bool:
        if self.messages is not None:
            self.logger.info("WaitForMessages: Received messages! Done!")
            return True
        if self.verbose:
            self.logger.info("WaitForMessages: waiting for messages from {}".format(self.topics))
        _dt = self.node.get_clock().now() - self._start_time
        if self.timeout is not None and _dt.nanoseconds * 1e-9 > self.timeout:
            self.logger.error("WaitForMessages: timeout waiting for messages")
            self.messages = (None,) * len(self.topics)
            self.has_timed_out = True
            if self.exception_on_timeout:
                raise TimeoutError("WaitForMessages: timeout waiting for messages")
        return False

    def _cb(self, *messages: typing.Any) -> None:
        if self.messages is not None:
            return
        if self.verbose:
            self.logger.info("WaitForMessages: callback got messages!")
        self.messages = messages

    def destroy_subs(self) -> None:
        """destroy all message filter subscribers"""
        if self.subs is not None:
            for mf_sub in self.subs:
                self.node.destroy_subscription(mf_sub.sub)
            self.subs = None
