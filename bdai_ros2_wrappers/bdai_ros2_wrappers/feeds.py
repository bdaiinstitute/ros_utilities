# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

from typing import Any, Callable, Iterator, List, Optional

from message_filters import SimpleFilter
from rclpy.node import Node
from rclpy.task import Future

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.utilities import Tape


class MessageFeed:
    """An ergonomic wrapper for generic message filters."""

    def __init__(
        self,
        link: SimpleFilter,
        *,
        history_length: Optional[int] = None,
        node: Optional[Node] = None,
    ) -> None:
        """Initializes the message feed.

        Args:
            link: Wrapped message filter, connecting this message feed with its source.
            history_length: optional historic data size, defaults to 1
            node: optional node for the underlying native subscription, defaults to
            the current process node.
        """
        if node is None:
            node = scope.ensure_node()
        if history_length is None:
            history_length = 1
        self._link = link
        self._tape = Tape(history_length)
        self._link.registerCallback(self._tape.write)
        node.context.on_shutdown(self._tape.close)

    @property
    def link(self) -> SimpleFilter:
        """Gets the underlying message connection."""
        return self._link

    @property
    def history(self) -> List[Any]:
        """Gets the entire history of messages received so far."""
        return list(self._tape.content())

    @property
    def latest(self) -> Optional[Any]:
        """Gets the latest message received, if any."""
        return next(self._tape.content(), None)

    @property
    def update(self) -> Future:
        """Gets the future to the next message yet to be received."""
        return self._tape.future_write

    def matching_update(self, matching_predicate: Callable[[Any], bool]) -> Future:
        """Gets a future to the next matching message yet to be received.

        Args:
            matching_predicate: a boolean predicate to match incoming messages.

        Returns:
            a future.
        """
        return self._tape.future_matching_write(matching_predicate)

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
        return self._tape.content(
            follow=True,
            forward_only=forward_only,
            buffer_size=buffer_size,
            timeout_sec=timeout_sec,
        )

    def close(self) -> None:
        """Closes the message feed."""
        self._tape.close()
