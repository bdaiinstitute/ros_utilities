# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from threading import Thread
from typing import Any, Optional

from rclpy import Context, Future
from rclpy.callback_groups import CallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, ShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from bdai_ros2_wrappers.callback_groups import NonReentrantCallbackGroup
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
    """An rclpy.node.Node subclass that changes the default callback group to be non-reentrant."""

    def __init__(self, *args: Any, default_callback_group: Optional[CallbackGroup] = None, **kwargs: Any):
        if default_callback_group is None:
            default_callback_group = NonReentrantCallbackGroup()
        self._default_callback_group_override = default_callback_group
        super().__init__(*args, **kwargs)

    @property
    def default_callback_group(self) -> CallbackGroup:
        """
        Get the default callback group.
        """
        # NOTE(hidmic): this overrides the hardcoded default group in rclpy.node.Node implementation
        return self._default_callback_group_override
