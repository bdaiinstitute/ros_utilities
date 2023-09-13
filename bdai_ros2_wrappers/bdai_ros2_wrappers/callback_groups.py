# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import threading
import typing
import weakref

import rclpy.callback_groups
import rclpy.executors


class NonReentrantCallbackGroup(rclpy.callback_groups.CallbackGroup):
    """
    A callback group to prevent concurrent execution of the same callback.

    Concurrent execution of different callbacks is allowed.
    """

    def __init__(self) -> None:
        super().__init__()
        self._active_entities: typing.Set[rclpy.executors.WaitableEntityType] = set()
        self._lock = threading.Lock()

    def can_execute(self, entity: rclpy.executors.WaitableEntityType) -> bool:
        with self._lock:
            assert weakref.ref(entity) in self.entities
            return entity not in self._active_entities

    def beginning_execution(self, entity: rclpy.executors.WaitableEntityType) -> bool:
        with self._lock:
            assert weakref.ref(entity) in self.entities
            if entity not in self._active_entities:
                self._active_entities.add(entity)
                return True
        return False

    def ending_execution(self, entity: rclpy.executors.WaitableEntityType) -> None:
        with self._lock:
            assert entity in self._active_entities
            self._active_entities.remove(entity)
