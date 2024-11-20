# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

import collections
import functools
import itertools
import threading
from collections.abc import Sequence
from typing import Any, Callable, Dict, Optional, Protocol, Tuple

import message_filters
import rclpy.subscription
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time

from bdai_ros2_wrappers.logging import RcutilsLogger


class SimpleFilterProtocol(Protocol):
    """Protocol for `message_filters.SimpleFilter` subclasses."""

    def registerCallback(self, callback: Callable, *args: Any) -> int:
        """Register callable to be called on filter output."""

    def signalMessage(self, *messages: Any) -> None:
        """Feed one or more `messages` to the filter."""


class Filter(SimpleFilterProtocol):
    """A threadsafe `message_filters.SimpleFilter` compliant message filter."""

    def __init__(self, autostart: bool = True) -> None:
        """Initialize filter.

        Args:
            autostart: whether to start filtering on instantiation or not.
        """
        self._active = False
        self._connection_lock = threading.Lock()
        self._connection_sequence = itertools.count()
        self.callbacks: Dict[int, Tuple[Callable, Tuple]] = {}
        if autostart:
            self.start()

    def _start(self) -> None:
        """Hook for start logic customization"""

    def start(self) -> None:
        """Start filtering."""
        with self._connection_lock:
            if not self._active:
                self._start()
                self._active = True

    def _stop(self) -> None:
        """Hook for stop logic customization"""

    def stop(self) -> None:
        """Stop filtering."""
        with self._connection_lock:
            if not self._active:
                self._stop()
                self._active = False

    def registerCallback(self, fn: Callable, *args: Any) -> int:
        """Register callable to be called on filter output.

        Args:
            fn: callback callable.
            args: optional positional arguments to supply on call.

        Returns:
            a unique connection identifier.
        """
        with self._connection_lock:
            connection = next(self._connection_sequence)
            self.callbacks[connection] = (fn, args)
            return connection

    def unregisterCallback(self, connection: int) -> None:
        """Unregister a callback.

        Args:
            connection: unique identifier for the callback.
        """
        with self._connection_lock:
            del self.callbacks[connection]

    def signalMessage(self, *messages: Any) -> None:
        """Feed one or more `messages` to the filter.

        Args:
            messages: messages to be forwarded through the filter.

        Raises:
            RuntimeError: if filter has not been started yet.
        """
        with self._connection_lock:
            if not self._active:
                raise RuntimeError("filter not started")
            callbacks = list(self.callbacks.values())

        for fn, args in callbacks:
            fn(*(messages + args))


class Subscriber(Filter):
    """A threadsafe `message_filters.Subscriber` equivalent."""

    def __init__(self, node: Node, *args: Any, autostart: bool = True, **kwargs: Any) -> None:
        """Initializes the `Subscriber` instance.

        Args:
            node: ROS 2 node to subscribe with.
            args: positional arguments to forward to `rclpy.node.Node.create_subscription`.
            autostart: whether to start filtering on instantiation or not.
            kwargs: keyword arguments to forward to `rclpy.node.Node.create_subscription`.
        """
        self.node = node
        self.options = (args, kwargs)
        self._subscription: Optional[rclpy.subscription.Subscription] = None
        super().__init__(autostart)

    def _start(self) -> None:
        if self._subscription is not None:
            raise RuntimeError("subscriber already subscribed")
        args, kwargs = self.options
        self._subscription = self.node.create_subscription(
            *args,
            callback=self.signalMessage,
            **kwargs,
        )

    def _stop(self) -> None:
        if self._subscription is None:
            raise RuntimeError("subscriber not subscribed")
        self.node.destroy_subscription(self._subscription)
        self._subscription = None

    close = Filter.stop

    def __getattr__(self, name: str) -> Any:
        return getattr(self._subscription, name)


class ApproximateTimeSynchronizer(Filter):
    """A threadsafe `message_filters.ApproximateTimeSynchronizer` equivalent."""

    def __init__(self, upstreams: Sequence[Filter], *args: Any, autostart: bool = True, **kwargs: Any) -> None:
        """Initializes the `ApproximateTimeSynchronizer` instance.

        Args:
            upstreams: message filters to be synchronized.
            args: positional arguments to forward to `message_filters.ApproximateTimeSynchronizer`.
            autostart: whether to start filtering on instantiation or not.
            kwargs: keyword arguments to forward to `message_filters.ApproximateTimeSynchronizer`.
        """
        self.upstreams = list(upstreams)
        self.options = (args, kwargs)
        self._unsafe_synchronizer: Optional[message_filters.ApproximateTimeSynchronizer] = None
        super().__init__(autostart)

    def _start(self) -> None:
        if self._unsafe_synchronizer is not None:
            raise RuntimeError("synchronizer already connected")
        args, kwargs = self.options
        self._unsafe_synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.upstreams,
            *args,
            **kwargs,
        )
        self._unsafe_synchronizer.registerCallback(self.signalMessage)
        for upstream in self.upstreams:
            upstream.start()

    def _stop(self) -> None:
        if self._unsafe_synchronizer is None:
            raise RuntimeError("synchronizer not connected")
        for upstream in self.upstreams:
            upstream.stop()
        self._unsafe_synchronizer = None

    def __getattr__(self, name: str) -> Any:
        return getattr(self._unsafe_synchronizer, name)


class TransformFilter(Filter):
    """A :mod:`tf2_ros` driven message filter, ensuring user defined transforms' availability.

    This filter passes a stamped transform along with filtered messages, from message frame ID
    to the given target frame ID, looked up at the message timestamp. The message is assumed
    to have a header. When filtering message tuples, only the first message header is observed.
    """

    def __init__(
        self,
        upstream: Filter,
        target_frame_id: str,
        tf_buffer: tf2_ros.Buffer,
        tolerance_sec: float,
        logger: Optional[RcutilsLogger] = None,
        autostart: bool = True,
    ) -> None:
        """Initializes the transform filter.

        Args:
            upstream: the upstream message filter.
            target_frame_id: the target frame ID for transforms.
            tf_buffer: a buffer of transforms to look up.
            tolerance_sec: a tolerance, in seconds, to wait for late transforms
            before abandoning any waits and filtering out the corresponding messages.
            logger: an optional logger to notify the yser about any errors during filtering.
            autostart: whether to start filtering on instantiation or not.
        """
        self.__lock = threading.RLock()
        self._logger = logger
        self._waitqueue: collections.deque = collections.deque()
        self._ongoing_wait: Optional[Future] = None
        self._ongoing_wait_time: Optional[Time] = None
        self._connection: Optional[int] = None
        self.target_frame_id = target_frame_id
        self.tf_buffer = tf_buffer
        self.tolerance = Duration(seconds=tolerance_sec)
        self.upstream = upstream
        super().__init__(autostart)

    def _start(self) -> None:
        if self._connection is not None:
            raise RuntimeError("filter already connected")
        self._connection = self.upstream.registerCallback(self.add)
        self.upstream.start()

    def _stop(self) -> None:
        if self._connection is None:
            raise RuntimeError("filter not connected")
        self.upstream.stop()
        self.upstream.unregisterCallback(self._connection)
        self._connection = None

    def _wait_callback(self, messages: Sequence[Any], future: Future) -> None:
        if future.cancelled():
            return
        with self.__lock:
            try:
                if future.result() is True:
                    source_frame_id = messages[0].header.frame_id
                    time = Time.from_msg(messages[0].header.stamp)
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame_id,
                        source_frame_id,
                        time,
                    )
                    self.signalMessage(*messages, transform)
            except tf2_ros.TransformException as e:
                if self._logger is not None:
                    self._logger.error(
                        (
                            "Got an exception during transform lookup: %s",
                            str(e),
                        ),
                    )

            if self._waitqueue:
                messages = self._waitqueue.popleft()
                source_frame_id = messages[0].header.frame_id
                time = Time.from_msg(messages[0].header.stamp)
                self._ongoing_wait = self.tf_buffer.wait_for_transform_async(
                    self.target_frame_id,
                    source_frame_id,
                    time,
                )
                self._ongoing_wait_time = time
                self._ongoing_wait.add_done_callback(
                    functools.partial(self._wait_callback, messages),
                )
            else:
                self._ongoing_wait_time = None
                self._ongoing_wait = None

    def add(self, *messages: Any) -> None:
        """Add `messages` to the filter."""
        with self.__lock:
            time = Time.from_msg(messages[0].header.stamp)
            if self._ongoing_wait and not self._ongoing_wait.done() and time - self._ongoing_wait_time > self.tolerance:
                self._ongoing_wait.cancel()
                self._ongoing_wait = None
            while self._waitqueue:
                pending_messages = self._waitqueue[0]
                pending_time = Time.from_msg(pending_messages[0].header.stamp)
                if time - pending_time <= self.tolerance:
                    break
                self._waitqueue.popleft()
            self._waitqueue.append(messages)
            if not self._ongoing_wait:
                messages = self._waitqueue.popleft()
                source_frame_id = messages[0].header.frame_id
                time = Time.from_msg(messages[0].header.stamp)
                self._ongoing_wait = self.tf_buffer.wait_for_transform_async(
                    self.target_frame_id,
                    source_frame_id,
                    time,
                )
                self._ongoing_wait_time = time
                self._ongoing_wait.add_done_callback(
                    functools.partial(self._wait_callback, messages),
                )


class Adapter(Filter):
    """A message filter for data adaptation."""

    def __init__(self, upstream: Filter, fn: Callable, autostart: bool = True) -> None:
        """Initializes the adapter.

        Args:
            upstream: the upstream message filter.
            fn: a callable that takes messages as arguments and returns some
            data to be signaled (i.e. propagated down the filter chain).
            If none is returned, no message signaling will occur.
            autostart: whether to start filtering on instantiation or not.
        """
        self.fn = fn
        self.upstream = upstream
        self._connection: Optional[int] = None
        super().__init__(autostart)

    def _start(self) -> None:
        if self._connection is not None:
            raise RuntimeError("adapter already connected")
        self._connection = self.upstream.registerCallback(self.add)
        self.upstream.start()

    def _stop(self) -> None:
        if self._connection is None:
            raise RuntimeError("adapter not connected")
        self.upstream.stop()
        self.upstream.unregisterCallback(self._connection)
        self._connection = None

    def add(self, *messages: Any) -> None:
        """Add `messages` to the filter."""
        result = self.fn(*messages)
        if result is not None:
            self.signalMessage(result)


class Tunnel(Filter):
    """A message filter that simply forwards messages but can be detached."""

    def __init__(self, upstream: Filter, autostart: bool = True) -> None:
        """Initializes the tunnel.

        Args:
            upstream: the upstream message filter.
            autostart: whether to start filtering on instantiation or not.
        """
        self.upstream: Optional[Filter] = upstream
        self._connection: Optional[int] = None
        super().__init__(autostart)

    def _start(self) -> None:
        if self.upstream is None:
            raise RuntimeError("tunnel closed")
        if self._connection is not None:
            raise RuntimeError("tunnel already connected")
        self._connection = self.upstream.registerCallback(self.signalMessage)
        self.upstream.start()

    def _stop(self) -> None:
        if self.upstream is None:
            raise RuntimeError("tunnel closed")
        if self._connection is None:
            raise RuntimeError("tunnel not connected")
        self.upstream.stop()
        self.upstream.unregisterCallback(self._connection)
        self._connection = None

    def close(self) -> None:
        """Closes the tunnel, simply disconnecting it from upstream."""
        with self._connection_lock:
            if self._connection is not None and self.upstream is not None:
                self.upstream.unregisterCallback(self._connection)
            self.upstream = None
