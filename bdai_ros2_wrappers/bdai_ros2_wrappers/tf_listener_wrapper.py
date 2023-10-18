# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

from typing import Optional

from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Duration, Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.futures import wait_for_future


class TFListenerWrapper:
    def __init__(self, node: Optional[Node] = None, cache_time_s: Optional[float] = None) -> None:
        node = node or scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._node = node
        cache_time_py = None
        if cache_time_s is not None:
            cache_time_py = Duration(nanoseconds=cache_time_s * 1e9)
        self._tf_buffer = Buffer(cache_time=cache_time_py)
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

    @property
    def buffer(self) -> Buffer:
        return self._tf_buffer

    def shutdown(self) -> None:
        self._tf_listener.unregister()

    def wait_for_a_tform_b_async(self, frame_a: str, frame_b: str, transform_time: Optional[Time] = None) -> Future:
        if transform_time is None:
            transform_time = Time()
        return self._tf_buffer.wait_for_transform_async(frame_a, frame_b, transform_time)

    def wait_for_a_tform_b(
        self, frame_a: str, frame_b: str, transform_time: Optional[Time] = None, timeout_sec: Optional[float] = None
    ) -> bool:
        """
        Wait for the transform from from_frame to to_frame to become available.
        """
        future = self.wait_for_a_tform_b_async(frame_a, frame_b, transform_time)
        if not wait_for_future(future, timeout_sec, context=self._node.context):
            future.cancel()
            return False
        return True

    def lookup_a_tform_b(
        self,
        frame_a: str,
        frame_b: str,
        transform_time: Optional[Time] = None,
        timeout_sec: Optional[float] = None,
        wait_for_frames: bool = False,
    ) -> TransformStamped:
        """
        Parameters:
            frame_a: Base frame for transform. The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform. The transform returned will be frame_a_t_frame_b
            transform_time: The time at which to look up the transform. If left at None, the most
                recent transform available will used.
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
                latest_time = Time(nanoseconds=latest_time.nanoseconds, clock_type=ClockType.ROS_TIME)
                if transform_time is not None and latest_time > transform_time:
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
        self, frame_a: str, frame_b: str, timeout_sec: Optional[float] = None, wait_for_frames: bool = False
    ) -> Time:
        """
        Parameters:
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
        transform = self.lookup_a_tform_b(frame_a, frame_b, timeout_sec, wait_for_frames)
        return Time.from_msg(transform.header.stamp)
