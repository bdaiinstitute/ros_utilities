# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
import threading
import typing
import weakref

import rclpy.callback_groups
import rclpy.executors


class NonReentrantCallbackGroup(rclpy.callback_groups.CallbackGroup):
    """A callback group to prevent concurrent execution of the same callback while allowing it for different callbacks.

    Note this behavior sits in between that offered by
    rclpy.callback_groups.MutuallyExclusiveCallbackGroup and
    rclpy.callback_groups.ReentrantCallbackGroup, as the former forbids
    concurrent execution and the latter allows it including multiple
    invocations of the same callback (e.g. a subscription handling multiple
    messages concurrently).

    See rclpy.callback_groups.CallbackGroup documentation for further reference.
    """

    def __init__(self) -> None:
        """Constructor"""
        super().__init__()
        self._active_entities: typing.Set[rclpy.executors.WaitableEntityType] = set()
        self._lock = threading.Lock()

    def can_execute(self, entity: rclpy.executors.WaitableEntityType) -> bool:
        """Determine if a callback for an entity can be executed.

        Args:
            entity: A subscription, timer, client, service, or waitable instance.

        Returns:
            `True` if entity callback can be executed, `False` otherwise.
        """
        with self._lock:
            assert weakref.ref(entity) in self.entities
            return entity not in self._active_entities

    def beginning_execution(self, entity: rclpy.executors.WaitableEntityType) -> bool:
        """Get permission for the callback from the group to begin executing an entity.

        If this returns `True` then `CallbackGroup.ending_execution` must be called after
        the callback has been executed.

        Arg:
            entity:A subscription, timer, client, service, or waitable instance.

        Returns:
            `True` if the callback can be executed, `False` otherwise.
        """
        with self._lock:
            assert weakref.ref(entity) in self.entities
            if entity not in self._active_entities:
                self._active_entities.add(entity)
                return True
        return False

    def ending_execution(self, entity: rclpy.executors.WaitableEntityType) -> None:
        """Notify group that a callback has finished executing.

        Args:
            entity: A subscription, timer, client, service, or waitable instance.
        """
        with self._lock:
            assert entity in self._active_entities
            self._active_entities.remove(entity)
