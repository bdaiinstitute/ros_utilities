# Copyright [2023] Boston Dynamics AI Institute, Inc.

import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy import Context
from rclpy.time import Duration, Time
from tf2_ros import ExtrapolationException, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from bdai_ros2_wrappers.node import NodeWrapper


class TFListenerWrapper(object):
    def __init__(
        self,
        node_name: str,
        wait_for_transform: Optional[Tuple[str, str]] = None,
        cache_time_s: Optional[int] = None,
        context: Optional[Context] = None,
    ) -> None:
        # private because we want to make sure no one but us spins this node!
        self._node = NodeWrapper(node_name=node_name, context=context, spin_thread=True)
        if cache_time_s is not None:
            cache_time_py = Duration(seconds=cache_time_s)
        else:
            cache_time_py = None
        self._tf_buffer = Buffer(cache_time=cache_time_py)
        self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)

        if wait_for_transform is not None and len(wait_for_transform) == 2:
            self.wait_for_init(wait_for_transform[0], wait_for_transform[1])

    def shutdown(self) -> None:
        """
        You must call this to have the program exit smoothly, unfortunately.  No del command seems to work.
        """
        self._node.shutdown()

    @property
    def buffer(self) -> Buffer:
        return self._tf_buffer

    def wait_for_init(self, from_frame: str, to_frame: str) -> None:
        """
        Waits for transform from from_frame to to_frame to become available and prints initializing statements.
        """
        # Wait for the buffer to fill to avoid annoying warnings.
        self._node.get_logger().info(
            "Waiting for TF to contain transform from " + str(from_frame) + " to " + str(to_frame)
        )
        self.wait_for_transform(from_frame, to_frame)
        self._node.get_logger().info("TF initialized")

    def wait_for_transform(self, from_frame: str, to_frame: str) -> None:
        """
        Wait for the transform from from_frame to to_frame to become available.
        """
        # We need a while instead of the usual wait because we use a different node and a thread for our listener
        while rclpy.ok(context=self._node.context):
            try:
                self.lookup_a_tform_b(from_frame, to_frame)
                break
            except TransformException:
                time.sleep(0.1)

    def _internal_lookup_a_tform_b(
        self,
        frame_a: str,
        frame_b: str,
        transform_time: Optional[float] = None,
        timeout: Optional[float] = None,
        wait_for_frames: bool = False,
    ) -> TransformStamped:
        """
        Parameters:
            frame_a: Base frame for transform.  The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform.  The transform returned will be frame_a_t_frame_b
            transform_time: The time at which to look up the transform.  If left at None, will be the most recent
                transform available
            timeout: The time to wait for the transform to become available if the transform_time is beyond the most
                recent transform in the buffer.
            wait_for_frames: If true, waits timeout amount of time for a path to exist from frame_a to frame_b in the
                buffer.  If false, this will return immediately if a path does not exist even if timeout is not
                None.  Note that wait_for_transform can be used to wait indefinitely for a transform to become
                available.
        Returns:
            The transform frame_a_t_frame_b at the time specified.
        Raises:
            All the possible TransformExceptions.
        """
        if transform_time is None:
            transform_time = Time()
        if timeout is None or not wait_for_frames:
            timeout_py = Duration()
        else:
            timeout_py = Duration(seconds=timeout)
        start_time = time.time()
        while rclpy.ok(context=self._node.context):
            try:
                return self._tf_buffer.lookup_transform(frame_a, frame_b, time=transform_time, timeout=timeout_py)
            except ExtrapolationException as e:
                if "future" not in str(e) or timeout is None:
                    raise e  # Waiting won't help with this
                now = time.time()
                if now - start_time > timeout:
                    raise e
                time.sleep(0.01)

    def lookup_a_tform_b(
        self,
        frame_a: str,
        frame_b: str,
        transform_time: Optional[float] = None,
        timeout: Optional[float] = None,
        wait_for_frames: bool = False,
    ) -> TransformStamped:
        """
        Parameters:
            frame_a: Base frame for transform.  The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform.  The transform returned will be frame_a_t_frame_b
            transform_time: The time at which to look up the transform.  If left at None, will be the most recent
                transform available
            timeout: The time to wait for the transform to become available if the transform_time is beyond the most
                recent transform in the buffer.
            wait_for_frames: If true, waits timeout amount of time for a path to exist from frame_a to frame_b in the
                buffer.  If false, this will return immediately if a path does not exist even if timeout is not
                None.  Note that wait_for_transform can be used to wait indefinitely for a transform to become
                available.
        Returns:
            The transform frame_a_t_frame_b at the time specified.
        Raises:
            All the possible TransformExceptions.
        """
        return self._internal_lookup_a_tform_b(frame_a, frame_b, transform_time, timeout, wait_for_frames)

    def lookup_latest_timestamp(
        self, frame_a: str, frame_b: str, timeout: Optional[float] = None, wait_for_frames: bool = False
    ) -> Time:
        """
        Parameters:
            frame_a: Base frame for transform.  The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform.  The transform returned will be frame_a_t_frame_b
            timeout: The time to wait for the transform to become available if the transform_time is beyond the most
                recent transform in the buffer.
            wait_for_frames: If true, waits timeout amount of time for a path to exist from frame_a to frame_b in the
                buffer.  If false, this will return immediately if a path does not exist even if timeout is not
                None.  Note that wait_for_transform can be used to wait indefinitely for a transform to become
                available.
        Returns:
            The timestamp from the latest recorded transform frame_a_t_frame_b
        Raises:
            All the possible TransformExceptions.
        """
        transform = self._internal_lookup_a_tform_b(frame_a, frame_b, None, timeout, wait_for_frames)
        return Time.from_msg(transform.header.stamp)
