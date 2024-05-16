# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import contextlib
from typing import Any, Iterable, Optional, cast

from rclpy.callback_groups import CallbackGroup
from rclpy.exceptions import InvalidHandle
from rclpy.node import Node as BaseNode
from rclpy.wait_set import WaitSet
from rclpy.waitables import Waitable

from bdai_ros2_wrappers.callback_groups import NonReentrantCallbackGroup
from bdai_ros2_wrappers.logging import MemoizingRcutilsLogger, as_memoizing_logger


class SafeWaitable:
    """A decorator to workaround https://github.com/ros2/rclpy/issues/1284."""

    def __init__(self, wrapped: Waitable) -> None:
        self._wrapped = wrapped

    def __getattr__(self, name: str) -> Any:
        return getattr(self._wrapped, name)

    def add_to_wait_set(self, wait_set: WaitSet) -> None:
        """Add waitable to `wait_set` safely."""
        with contextlib.suppress(InvalidHandle):
            self._wrapped.add_to_wait_set(wait_set)


class Node(BaseNode):
    """An rclpy.node.Node subclass that:

    * changes the default callback group to be non-reentrant
    * wraps its logger with a memoizing one for improved efficiency
    """

    def __init__(self, *args: Any, default_callback_group: Optional[CallbackGroup] = None, **kwargs: Any) -> None:
        """Initializes the node.

        Args:
            args: positional arguments for a ros Node
            default_callback_group: optional callback group to use as default
                for all subsequently created entities, such as subscriptions
                and clients.
            kwargs: keyword arguments for a ros Node

        See rclpy.node.Node documentation for further reference on available arguments.
        """
        if default_callback_group is None:
            default_callback_group = NonReentrantCallbackGroup()
        self._default_callback_group_override = default_callback_group
        self._destruction_requested = False
        super().__init__(*args, **kwargs)
        self._logger: MemoizingRcutilsLogger = as_memoizing_logger(self._logger)

    @property
    def default_callback_group(self) -> CallbackGroup:
        """Get the default callback group."""
        # NOTE(hidmic): this overrides the hardcoded default group in rclpy.node.Node implementation
        return self._default_callback_group_override

    @property
    def waitables(self) -> Iterable[Waitable]:
        """Get node waitables."""
        for waitable in super().waitables:
            yield cast(Waitable, SafeWaitable(waitable))

    @property
    def destruction_requested(self) -> bool:
        """Checks whether destruction was requested or not."""
        return self._destruction_requested

    def destroy_node(self) -> None:
        """Overrides node destruction API."""
        self._destruction_requested = True
        super().destroy_node()
