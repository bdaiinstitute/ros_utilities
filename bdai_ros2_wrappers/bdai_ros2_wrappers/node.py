# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from typing import Any, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node as BaseNode

from bdai_ros2_wrappers.callback_groups import NonReentrantCallbackGroup


class Node(BaseNode):
    """
    An rclpy.node.Node subclass that:

    * changes the default callback group to be non-reentrant.
    """

    def __init__(self, *args: Any, default_callback_group: Optional[CallbackGroup] = None, **kwargs: Any) -> None:
        """
        Initializes the node.

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
        super().__init__(*args, **kwargs)

    @property
    def default_callback_group(self) -> CallbackGroup:
        """Get the default callback group."""
        # NOTE(hidmic): this overrides the hardcoded default group in rclpy.node.Node implementation
        return self._default_callback_group_override
