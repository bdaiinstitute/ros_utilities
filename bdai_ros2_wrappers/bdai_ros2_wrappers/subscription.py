# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

from collections.abc import Sequence
from typing import Any, Optional, Type, Union, cast

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.task import Future

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.feeds import MessageFeed
from bdai_ros2_wrappers.filters import ApproximateTimeSynchronizer, Subscriber
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.type_hints import Msg as MessageT


class Subscription(MessageFeed):
    """An ergonomic interface for topic subscriptions in ROS 2.

    Subscription instances are `MessageFeed` instances wrapping `message_filters.Subscriber`
    instances and thus allow synchronous and asynchronous iteration and fetching of published data.
    """

    def __init__(
        self,
        message_type: Type[MessageT],
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
        if node is None:
            node = scope.ensure_node()
        if history_length is None:
            history_length = 1
        if qos_profile is None:
            qos_profile = 1
        super().__init__(
            Subscriber(
                node,
                message_type,
                topic_name,
                qos_profile=qos_profile,
                **kwargs,
            ),
            history_length=history_length,
            node=node,
        )
        self._message_type = message_type
        self._topic_name = topic_name
        self._node = node

    @property
    def subscriber(self) -> Subscriber:
        """Gets the underlying subscriber.

        Type-casted alias of `Subscription.link`.
        """
        return cast(Subscriber, self.link)

    def publisher_matches(self, num_publishers: int) -> Future:
        """Gets a future to next publisher matching status update.

        Note that in ROS 2 Humble and ealier distributions, this method relies on
        polling the number of known publishers for the topic subscribed, as subscription
        matching events are missing.

        Args:
            num_publishers: lower bound on the number of publishers to match.

        Returns:
            a future, done if the current number of publishers already matches
            the specified lower bound.
        """
        future_match = Future()
        num_matched_publishers = self._node.count_publishers(self._topic_name)
        if num_matched_publishers < num_publishers:

            def _poll_publisher_matches() -> None:
                nonlocal future_match, num_publishers
                if future_match.cancelled():
                    return
                num_matched_publishers = self._node.count_publishers(self._topic_name)
                if num_publishers <= num_matched_publishers:
                    future_match.set_result(num_matched_publishers)

            timer = self._node.create_timer(0.1, _poll_publisher_matches)
            future_match.add_done_callback(lambda _: self._node.destroy_timer(timer))
        else:
            future_match.set_result(num_matched_publishers)
        return future_match

    @property
    def matched_publishers(self) -> int:
        """Gets the number publishers matched and linked to.

        Note that in ROS 2 Humble and earlier distributions, this property
        relies on the number of known publishers for the topic subscribed
        as subscription matching status info is missing.
        """
        return self._node.count_publishers(self._topic_name)

    @property
    def message_type(self) -> Type[MessageT]:
        """Gets the type of the message subscribed."""
        return self._message_type

    @property
    def topic_name(self) -> str:
        """Gets the name of the topic subscribed."""
        return self._topic_name

    def close(self) -> None:
        """Closes the subscription."""
        self._node.destroy_subscription(self.subscriber.sub)
        super().close()

    # Aliases for improved readability
    cancel = MessageFeed.close
    unsubscribe = MessageFeed.close


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
    if node is None:
        node = scope.ensure_node()
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
    if node is None:
        node = scope.ensure_node()
    future = wait_for_message_async(msg_type, topic_name, node=node, **kwargs)
    if not wait_for_future(future, timeout_sec, context=node.context):
        future.cancel()
        return None
    return future.result()


def wait_for_messages(
    topic_names: Sequence[str],
    message_types: Sequence[MessageT],
    *,
    timeout_sec: Optional[float] = None,
    node: Optional[Node] = None,
    **kwargs: Any,
) -> Optional[Sequence[Any]]:
    """Waits for messages to arrive at multiple topics within a given time window.

    Uses message_filters.ApproximateTimeSynchronizer. This function blocks
    until receiving the messages or when a given timeout expires. Assumes the
    given node is spinning by some external executor.

    Requires the user to pass in a node, since
    message_filters.Subscriber requires a node upon construction.

    Args:
        topic_names: list of topic names
        message_types: list of message types, one for each topic.
        timeout_sec: optional time in seconds to wait. None if forever.
        node: optional node for temporary topic subscription
        kwargs: additional arguments passed into `wait_for_messages_async`.

    See `wait_for_messages_async` documentation for a reference on
    additional keyword arguments.
    """
    if node is None:
        node = scope.ensure_node()
    future = wait_for_messages_async(topic_names, message_types, node=node, **kwargs)
    if not wait_for_future(future, timeout_sec, context=node.context):
        future.cancel()
        return None
    return future.result()


def wait_for_messages_async(
    topic_names: Sequence[str],
    message_types: Sequence[MessageT],
    *,
    queue_size: int = 10,
    delay: float = 0.2,
    allow_headerless: bool = False,
    node: Optional[Node] = None,
    qos_profiles: Optional[Sequence[Optional[QoSProfile]]] = None,
    callback_group: Optional[CallbackGroup] = None,
) -> Future:
    """Asynchronous version of wait_for_messages

    Args:
        topic_names: list of topic names
        message_types: List of message types, one for each topic.
        queue_size: synchronizer message queue size
        delay: the delay in seconds for which the messages could be
        synchronized (i.e. the time window).
        allow_headerless: whether it's ok for there to be no header in the messages.
        qos_profiles: optional list of QoS profiles, one for each topic.
        If no QoS profile is specified for a given topic, the default profile with
        a history depth of 10 will be used.
        node: optional node for temporary topic subscription, defaults to the current
        process-wide node (if any).
        callback_group: optional callback group for the message filter subscribers.
    """
    if node is None:
        node = scope.ensure_node()
    if qos_profiles is None:
        qos_profiles = [None] * len(topic_names)

    subscribers = [
        Subscriber(
            node,
            message_type,
            topic_name,
            qos_profile=qos_profile or 10,
            callback_group=callback_group,
        )
        for topic_name, message_type, qos_profile in zip(topic_names, message_types, qos_profiles)
    ]

    future = Future()

    def callback(*messages: Sequence[Any]) -> None:
        nonlocal future
        if not future.done():
            future.set_result(messages)

    sync = ApproximateTimeSynchronizer(
        subscribers,
        queue_size,
        delay,
        allow_headerless,
    )
    sync.registerCallback(callback)

    def cleanup_subscribers(_: Future) -> None:
        nonlocal node, subscribers
        assert node is not None
        for sub in subscribers:
            node.destroy_subscription(sub.sub)

    future.add_done_callback(cleanup_subscribers)
    return future
