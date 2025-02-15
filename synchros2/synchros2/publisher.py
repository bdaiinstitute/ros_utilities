#  Copyright (c) 2025 Boston Dynamics AI Institute Inc.  All rights reserved.

from asyncio import Future
from typing import Any, Generic, Optional, Type, TypeVar

import rclpy.publisher
from rclpy.node import Node

import synchros2.scope as scope

# Represents a ros message type
_MessageT = TypeVar("_MessageT")


class Publisher(Generic[_MessageT]):
    """An extension of a publisher from ROS 2."""

    def __init__(self, *args: Any, node: Optional[Node] = None, **kwargs: Any) -> None:
        """Initializes the Publisher

        Args:
            args: Positional arguments to pass to the `Node.create_publisher` function
            node: Optional node for the underlying native subscription, defaults to
                the current process node.
            kwargs: Keyword arguments to pass to the `Node.create_publisher` function
        """
        if node is None:
            node = scope.ensure_node()
        self._node = node
        self._publisher = self._node.create_publisher(*args, **kwargs)

    def subscription_matches(self, num_subscriptions: int) -> Future:
        """Gets a future to next publisher matching status update.

        Note that in ROS 2 Humble and earlier distributions, this method relies on
        polling the number of known subscriptions for the topic subscribed, as publisher
        matching events are missing.

        Args:
            num_subscriptions: lower bound on the number of subscriptions to match.

        Returns:
            a future, done if the current number of subscriptions already matches
            the specified lower bound.
        """
        future_match = Future()  # type: ignore[var-annotated]
        num_matched_publishers = self._node.count_subscribers(self._publisher.topic_name)
        if num_matched_publishers < num_subscriptions:

            def _poll_publisher_matches() -> None:
                nonlocal future_match, num_subscriptions
                if future_match.cancelled():
                    return
                num_matched_subscriptions = self._node.count_subscribers(self._publisher.topic_name)
                if num_subscriptions <= num_matched_subscriptions:
                    future_match.set_result(num_matched_subscriptions)

            timer = self._node.create_timer(0.1, _poll_publisher_matches)
            future_match.add_done_callback(lambda _: self._node.destroy_timer(timer))
        else:
            future_match.set_result(num_matched_publishers)
        return future_match

    @property
    def matched_subscriptions(self) -> int:
        """Gets the number subscriptions matched and linked to.

        Note that in ROS 2 Humble and earlier distributions, this property
        relies on the number of known subscriptions for the topic subscribed
        as subscription matching status info is missing.
        """
        return self._node.count_subscribers(self._publisher.topic_name)

    @property
    def message_type(self) -> Type[_MessageT]:
        """Gets the type of the message subscribed."""
        return self._publisher.msg_type

    def publisher(self) -> rclpy.publisher.Publisher:
        """Returns the internal ROS 2 publisher"""
        return self._publisher

    def __getattr__(self, name: str) -> Any:
        return getattr(self._publisher, name)
