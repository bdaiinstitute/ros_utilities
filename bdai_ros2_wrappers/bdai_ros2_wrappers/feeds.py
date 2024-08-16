# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

from typing import Any, Callable, Generic, Iterable, Iterator, List, Optional, TypeVar

import tf2_ros
from rclpy.node import Node

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.filters import Adapter, ApproximateTimeSynchronizer, Filter, TransformFilter, Tunnel
from bdai_ros2_wrappers.futures import FutureLike
from bdai_ros2_wrappers.utilities import Tape

MessageT = TypeVar("MessageT")


class MessageFeed(Generic[MessageT]):
    """An ergonomic wrapper for generic message filters."""

    def __init__(
        self,
        link: Filter,
        *,
        history_length: Optional[int] = None,
        node: Optional[Node] = None,
    ) -> None:
        """Initializes the message feed.

        Args:
            link: Wrapped message filter, connecting this message feed with its source.
            history_length: optional historic data size, defaults to 1
            node: optional node for lifetime control, defaults to the current process node.
        """
        if node is None:
            node = scope.ensure_node()
        if history_length is None:
            history_length = 1
        self._link = link
        self._tape = Tape[MessageT](history_length)
        self._link.registerCallback(lambda *msgs: self._tape.write(msgs if len(msgs) > 1 else msgs[0]))
        node.context.on_shutdown(self._tape.close)

    @property
    def link(self) -> Filter:
        """Gets the underlying message connection."""
        return self._link

    @property
    def history(self) -> List[MessageT]:
        """Gets the entire history of messages received so far."""
        return list(self._tape.content())

    @property
    def latest(self) -> Optional[MessageT]:
        """Gets the latest message received, if any."""
        return self._tape.head

    @property
    def update(self) -> FutureLike[MessageT]:
        """Gets the future to the next message yet to be received."""
        return self._tape.future_write

    def matching_update(self, matching_predicate: Callable[[MessageT], bool]) -> FutureLike[MessageT]:
        """Gets a future to the next matching message yet to be received.

        Args:
            matching_predicate: a boolean predicate to match incoming messages.

        Returns:
            a future.
        """
        return self._tape.future_matching_write(matching_predicate)

    def recall(self, callback: Callable[[MessageT], None]) -> Tunnel:
        """Adds a callback for message recalling.

        Returns:
            the underlying connection, which can be closed to stop future callbacks.
        """
        tunnel = Tunnel(self.link)
        tunnel.registerCallback(callback)
        return tunnel

    def stream(
        self,
        *,
        forward_only: bool = False,
        buffer_size: Optional[int] = None,
        timeout_sec: Optional[float] = None,
    ) -> Iterator[MessageT]:
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


class AdaptedMessageFeed(MessageFeed[MessageT]):
    """A message feed decorator to simplify adapter patterns."""

    def __init__(
        self,
        feed: MessageFeed,
        fn: Callable[..., MessageT],
        **kwargs: Any,
    ) -> None:
        """Initializes the message feed.

        Args:
            feed: the upstream (ie. decorated) message feed.
            fn: message adapting callable.
            kwargs: all other keyword arguments are forwarded
            for `MessageFeed` initialization.
        """
        super().__init__(Adapter(feed.link, fn), **kwargs)
        self._feed = feed

    @property
    def feed(self) -> MessageFeed:
        """Gets the upstream message feed."""
        return self._feed

    def close(self) -> None:
        """Closes this message feed and the upstream one as well."""
        self._feed.close()
        super().close()


class FramedMessageFeed(MessageFeed[MessageT]):
    """A message feed decorator, incorporating transforms using a `TransformFilter` instance."""

    def __init__(
        self,
        feed: MessageFeed,
        target_frame_id: str,
        *,
        tolerance_sec: float = 1.0,
        tf_buffer: Optional[tf2_ros.Buffer] = None,
        history_length: Optional[int] = None,
        node: Optional[Node] = None,
    ) -> None:
        """Initializes the message feed.

        Args:
            feed: the upstream message feed.
            target_frame_id: the target frame ID for transforms.
            tf_buffer: optional buffer of transforms to look up. If none is provided
            a transforms' buffer and a listener will be instantiated.
            tolerance_sec: optional tolerance, in seconds, to wait for late transforms.
            history_length: optional historic data size, defaults to 1.
            node: optional node for the underlying native subscription, defaults to
            the current process node.
        """
        if node is None:
            node = scope.ensure_node()
        if tf_buffer is None:
            tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(tf_buffer, node)
        super().__init__(
            TransformFilter(
                feed.link,
                target_frame_id,
                tf_buffer,
                tolerance_sec,
                node.get_logger(),
            ),
            history_length=history_length,
            node=node,
        )
        self._feed = feed

    @property
    def feed(self) -> MessageFeed[MessageT]:
        """Gets the upstream message feed."""
        return self._feed

    def close(self) -> None:
        """Closes this message feed and the upstream one as well."""
        self._feed.close()
        super().close()


class SynchronizedMessageFeed(MessageFeed):
    """A message feeds' aggregator using a `message_filters.ApproximateTimeSynchronizer` instance."""

    def __init__(
        self,
        *feeds: MessageFeed,
        queue_size: int = 10,
        delay: float = 0.2,
        allow_headerless: bool = False,
        history_length: Optional[int] = None,
        node: Optional[Node] = None,
    ) -> None:
        """Initializes the message feed.

        Args:
            feeds: upstream message feeds to be synchronized.
            queue_size: the message queue size for synchronization.
            delay: the maximum delay, in seconds, between messages for synchronization to succeed.
            allow_headerless: whether it's OK for there to be no header in the messages (in which
            case, the ROS time of arrival will be used).
            history_length: optional historic data size, defaults to 1.
            node: optional node for the underlying native subscription, defaults to
            the current process node.
        """
        super().__init__(
            ApproximateTimeSynchronizer(
                [f.link for f in feeds],
                queue_size,
                delay,
                allow_headerless=allow_headerless,
            ),
            history_length=history_length,
            node=node,
        )
        self._feeds = feeds

    @property
    def feeds(self) -> Iterable[MessageFeed]:
        """Gets all aggregated message feeds."""
        return self._feeds

    def close(self) -> None:
        """Closes this message feed and all upstream ones as well."""
        for feed in self._feeds:
            feed.close()
        super().close()
