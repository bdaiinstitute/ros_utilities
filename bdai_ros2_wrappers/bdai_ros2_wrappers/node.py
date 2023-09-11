# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from threading import Thread
from typing import Any, Optional

from rclpy import Context, Future
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, ShutdownException, SingleThreadedExecutor
from rclpy.guard_condition import GuardCondition
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from bdai_ros2_wrappers.futures import wait_for_future


class NodeWrapper(Node):
    """A wrapper around a node and its executor."""

    def __init__(
        self,
        node_name: str,
        *,
        context: Optional[Context] = None,
        namespace: Optional[str] = None,
        spin_thread: bool = False,
        num_executor_threads: int = 1,
    ) -> None:
        """Constructor

        Args:
            node_name (str): The name of the node
            context (Optional[Context]): The context for the node
            namespace (Optional[str]): The namespace for the node
            spin_thread (bool): Flag on whether to start spinning the thread on construction
            num_executor_threads (int): Number of threads the executor should use.
                1 => Single Thread, >1 => Multithread of that number of executors,
                -1 => Multithread with as many threads as the system can do using `multiprocessing.cpu_count`
        """
        super().__init__(node_name, context=context, namespace=namespace)

        # Use single thread
        if num_executor_threads == 1:
            self._executor = SingleThreadedExecutor(context=context)
        # Use all available threads
        elif num_executor_threads == -1:
            self._executor = MultiThreadedExecutor(context=context)
        # Use specified number of threads
        else:
            self._executor = MultiThreadedExecutor(context=context, num_threads=num_executor_threads)
        self._executor.add_node(self)

        if spin_thread:
            self._thread: Optional[Thread] = Thread(target=self._spin)
            self._thread.start()
        else:
            self._thread = None

    def _spin(self) -> None:
        """Internal function to spin the executor"""
        try:
            self._executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException, ShutdownException):
            pass

    def spin_until_future_complete(self, future: Future, timeout_sec: Optional[float] = None) -> bool:
        """Spin the internal node until the future completes, and return the result. This is safe to call from a
        callback.

        Args:
            future (Future): An object that encapsulates some of the asynchronous execution of some callable
            timeout_sec (Optional[float]): An optional timeout for how long to spin the executor

        Returns:
            bool: True if successful, False if the timeout was triggered
        """
        if self._thread is not None:
            return wait_for_future(future, timeout_sec=timeout_sec)
        self._executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        return future.done()

    def shutdown(self) -> None:
        """Shuts down the internal executor and spin thread"""
        if self._executor is not None:
            self._executor.shutdown()
            self._executor.remove_node(self)
            self._executor = None
        if self._thread is not None:
            self._thread.join()
        self.destroy_node()

    # def __del__(self) -> None:
    #     """Destructor"""
    #     self.shutdown()


class FriendlyNode(Node):
    def __init__(self, *args: Any, enable_callback_isolation: bool = True, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self._enable_callback_isolation = enable_callback_isolation

    @property
    def enable_callback_isolation(self) -> bool:
        return self._enable_callback_isolation

    def create_subscription(
        self, *args: Any, callback_group: Optional[CallbackGroup] = None, **kwargs: Any
    ) -> Subscription:
        if callback_group is None and self._enable_callback_isolation:
            callback_group = MutuallyExclusiveCallbackGroup()
        return super().create_subscription(*args, callback_group=callback_group, **kwargs)

    def create_publisher(self, *args: Any, callback_group: Optional[CallbackGroup] = None, **kwargs: Any) -> Publisher:
        if callback_group is None and self._enable_callback_isolation:
            callback_group = MutuallyExclusiveCallbackGroup()
        return super().create_publisher(*args, callback_group=callback_group, **kwargs)

    def create_client(self, *args: Any, callback_group: Optional[CallbackGroup] = None, **kwargs: Any) -> Client:
        if callback_group is None and self._enable_callback_isolation:
            callback_group = MutuallyExclusiveCallbackGroup()
        return super().create_client(*args, callback_group=callback_group, **kwargs)

    def create_service(self, *args: Any, callback_group: Optional[CallbackGroup] = None, **kwargs: Any) -> Service:
        if callback_group is None and self._enable_callback_isolation:
            callback_group = MutuallyExclusiveCallbackGroup()
        return super().create_service(*args, callback_group=callback_group, **kwargs)

    def create_timer(self, *args: Any, callback_group: Optional[CallbackGroup] = None, **kwargs: Any) -> Timer:
        if callback_group is None and self._enable_callback_isolation:
            callback_group = MutuallyExclusiveCallbackGroup()
        return super().create_timer(*args, callback_group=callback_group, **kwargs)

    def create_guard_condition(
        self, *args: Any, callback_group: Optional[CallbackGroup] = None, **kwargs: Any
    ) -> GuardCondition:
        if callback_group is None and self._enable_callback_isolation:
            callback_group = MutuallyExclusiveCallbackGroup()
        return super().create_guard_condition(*args, callback_group=callback_group, **kwargs)
