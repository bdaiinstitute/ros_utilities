# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

import warnings
from typing import Optional, Protocol, Union, runtime_checkable

from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Duration, Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from synchros2.futures import wait_for_future


@runtime_checkable
class StampLike(Protocol):
    """A protocol similar to the Time msg"""

    sec: int
    nanosec: int


@runtime_checkable
class TimeLike(Protocol):
    """A protocol similar to to rclpy.time.Time"""

    nanoseconds: int


def from_time_like(obj: Union[StampLike, TimeLike]) -> Time:
    """Convert an object that exhibits a Time-like interface to a proper Time instance."""
    if isinstance(obj, StampLike):
        return Time.from_msg(obj, clock_type=ClockType.SYSTEM_TIME)
    if isinstance(obj, TimeLike):
        return Time(nanoseconds=obj.nanoseconds, clock_type=ClockType.SYSTEM_TIME)
    raise TypeError(f"{obj} does not define a point in time")


class TFListenerWrapper:
    """A ``tf2_ros`` lookup device, wrapping both a buffer and a listener.

    When using process-wide machinery:

    .. code-block:: python

        tf_listener = TFListenerWrapper()
        tf_listener.wait_for_a_tform_b(target_frame, source_frame)
        tf_listener.lookup_a_tform_b(target_frame, source_frame)

    When composed by a ROS 2 node:

    .. code-block:: python

        class MyNode(Node):

            def __init__(self, **kwargs: Any) -> None:
                super().__init__("my_node", **kwargs)
                self.tf_listener = TFListenerWrapper(self)
                # do not wait synchronously here or it will block forever!
                # ...

            def callback(self):
                if tf_listener.wait_for_a_tform_b(target_frame, source_frame, timeout_sec=0.0):
                    self.tf_listener.lookup_a_tform_b(target_frame, source_frame)
                    # or you can lookup and handle exceptions yourself
    """

    def __init__(self, node: Optional[Node] = None, cache_time_s: Optional[float] = None) -> None:
        """Initializes the wrapper.

        Args:
            node: optional node for transform listening, defaults to the current scope node.
            cache_time_s: optional transform buffer size, in seconds.
        """
        import synchros2.scope as scope  # locally to avoid circular import

        if node is None:
            node = scope.ensure_node()
        self._node = node
        cache_time_py = None
        if cache_time_s is not None:
            cache_time_py = Duration(nanoseconds=cache_time_s * 1e9)
        self._tf_buffer = Buffer(cache_time=cache_time_py)
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

    @property
    def buffer(self) -> Buffer:
        """Returns the tf buffer"""
        return self._tf_buffer

    def shutdown(self) -> None:
        """Shutdown the tf listener"""
        self._tf_listener.unregister()

    def wait_for_a_tform_b_async(
        self,
        frame_a: str,
        frame_b: str,
        transform_time: Optional[Union[StampLike, TimeLike]] = None,
    ) -> Future:
        """Wait asynchronously for the transform from from_frame to to_frame to become available.

        Args:
            frame_a: Base frame for transform. The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform. The transform returned will be frame_a_t_frame_b
            transform_time: The time at which to look up the transform. If left at None, the most
            recent transform available will used.

        Returns:
            A future that will tell if a transform between frame_a and frame_b is available at the time specified.
        """
        if transform_time is None:
            transform_time = Time()
        return self._tf_buffer.wait_for_transform_async(frame_a, frame_b, transform_time)

    def wait_for_a_tform_b(
        self,
        frame_a: str,
        frame_b: str,
        transform_time: Optional[Union[StampLike, TimeLike]] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """Wait for a transform from frame_a to frame_b to become available.

        Note this is a blocking call. If the underlying node is not spinning, an indefinite wait may block forever.

        Args:
            frame_a: Base frame for transform. The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform. The transform returned will be frame_a_t_frame_b
            transform_time: The time at which to look up the transform. If left at None, the most
            recent transform available will used.
            timeout_sec: The time to wait for the transform to become available if the requested time is beyond
            the most recent transform in the buffer. If set to 0, it will not wait. If left at None, it will
            wait indefinitely.

        Returns:
            Whether a transform between frame_a and frame_b is available at the specified time.
        """
        if self._node.executor is None:
            if timeout_sec is None:
                warnings.warn("Node is not spinning yet, wait may block forever", stacklevel=1)
            elif timeout_sec > 0.0:
                warnings.warn("Node is not spinning yet, wait may be futile", stacklevel=1)
        future = self.wait_for_a_tform_b_async(frame_a, frame_b, transform_time)
        if not wait_for_future(future, timeout_sec, context=self._node.context):
            future.cancel()
            return False
        return True

    def lookup_a_tform_b(
        self,
        frame_a: str,
        frame_b: str,
        transform_time: Optional[Union[StampLike, TimeLike]] = None,
        timeout_sec: Optional[float] = None,
        wait_for_frames: bool = False,
    ) -> TransformStamped:
        """Looks up the transform from frame_a to frame_b at the specified time.

        Args:
            frame_a: Base frame for transform. The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform. The transform returned will be frame_a_t_frame_b
            transform_time: The time at which to look up the transform. If left at None,
            the most recent transform available will used.
            timeout_sec: The time to wait for the transform to become available if the requested time is beyond
            the most recent transform in the buffer. If set to 0, it will not wait. If left at None, it will
            wait indefinitely.
            wait_for_frames: If true, it will wait for a path to exist from frame_a to frame_b in the
            buffer. If false, lookup will fail immediately if a path between frames does not exist,
            regardless of what timeout was set. Note that wait_for_a_tform_b can also be used to
            wait for a transform to become available.

        Returns:
            The transform frame_a_t_frame_b at the time specified.

        Raises:
            All the possible TransformExceptions.
        """
        if not wait_for_frames:
            try:
                latest_time = self._tf_buffer.get_latest_common_time(frame_a, frame_b)
                # tf2 Python bindings yield system clock timestamps rather than ROS clock
                # timestamps, regardless of where these timestamps came from (as there is no
                # clock notion in tf2)
                if transform_time is not None:
                    transform_time = from_time_like(transform_time)
                    if latest_time > transform_time:
                        # no need to wait, either lookup can be satisfied or an extrapolation
                        # to the past exception will ensue
                        timeout_sec = 0.0
            except TransformException:
                # either frames do not exist or a path between them doesn't, do not wait
                timeout_sec = 0.0
        if transform_time is None:
            transform_time = Time()
        if timeout_sec != 0.0:
            self.wait_for_a_tform_b(frame_a, frame_b, transform_time, timeout_sec)
        return self._tf_buffer.lookup_transform(frame_a, frame_b, transform_time)

    def lookup_latest_timestamp(
        self,
        frame_a: str,
        frame_b: str,
        timeout_sec: Optional[float] = None,
        wait_for_frames: bool = False,
    ) -> Time:
        """Looks up the latest time at which a transform from frame_a to frame_b is available.

        Args:
            frame_a: Base frame for transform.  The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform.  The transform returned will be frame_a_t_frame_b
            timeout_sec: The time to wait for the transform to become available if the requested time is beyond
            the most recent transform in the buffer. If set to 0, it will not wait. If left at None, it will
            wait indefinitely.
            wait_for_frames: If true, it will wait for a path to exist from frame_a to frame_b in the
            buffer. If false, lookup will fail immediately if a path between frames does not exist,
            regardless of what timeout was set. Note that wait_for_a_tform_b can also be used to
            wait for a transform to become available.

        Returns:
            The timestamp from the latest recorded transform frame_a_t_frame_b

        Raises:
            All the possible TransformExceptions.
        """
        transform = self.lookup_a_tform_b(frame_a, frame_b, Time(), timeout_sec, wait_for_frames)
        return Time.from_msg(transform.header.stamp)
