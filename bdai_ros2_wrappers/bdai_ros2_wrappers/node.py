# Copyright (c) 2023 Boston Dynamics AI Institute, Inc.  All rights reserved.
from threading import Thread
from typing import Any, Optional

from rclpy import Context, Future
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from bdai_ros2_wrappers.future import wait_until_future_complete


class NodeWrapper(Node):
    """A wrapper around a node and its executor.

    @param num_executor_threads: Number of threads the executor should use.
                        1 => Single Thread, >1 => Multithread of that number of executors,
                        -1 => Multithread with as many threads as the system can do using `multiprocessing.cpu_count`

    @param spin_thread: Flag on whether to start spinning the thread on construction
    """

    def __init__(
        self,
        node_name: str,
        *,
        context: Optional[Context] = None,
        namespace: Optional[str] = None,
        spin_thread: bool = False,
        num_executor_threads: int = 1,
    ) -> None:
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
        try:
            self._executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    def spin_until_future_complete(self, future: Future, timeout_sec: Optional[float] = None) -> Any:
        """Spin the internal node until the future completes, and return the result. This is safe to call from a
        callback."""
        if self._thread is not None:
            wait_until_future_complete(future, timeout_sec=timeout_sec)
        else:
            self._executor.spin_until_future_complete(future, timeout_sec=timeout_sec)

        return future.result()

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._executor.remove_node(self)
        if self._thread is not None:
            self._thread.join()
        self.destroy_node()
