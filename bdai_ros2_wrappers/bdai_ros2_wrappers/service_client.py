# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from typing import Optional

from rclpy import Context
from rclpy.service import SrvType, SrvTypeRequest, SrvTypeResponse

from bdai_ros2_wrappers.node import NodeWrapper


class ServiceClientWrapper:
    """A wrapper around a service client that allows the service to be safely called from callbacks without
    deadlocking."""

    def __init__(
        self,
        node_name: str,
        service_type: SrvType,
        service_name: str,
        context: Optional[Context] = None,
        spin_thread: bool = False,
        num_executor_threads: int = 1,
    ) -> None:
        """Constructor

        Args:
            node_name (str): The name of internal node
            service_type (SrvType): The type of the service
            service_name (str): The name of the service
            context (Optional[Context]): The context of the internal node
            spin_thread (bool): Flag on whether to start a spinning thread on the internal node
            num_executor_threads (int): Number of threads the executor should use.
                1 => Single Thread, >1 => Multithread of that number of executors,
                -1 => Multithread with as many threads as the system can do using `multiprocessing.cpu_count`
        """
        self._node_wrapper = NodeWrapper(
            node_name, context=context, spin_thread=spin_thread, num_executor_threads=num_executor_threads
        )
        self._service_name = service_name
        self._client = self._node_wrapper.create_client(service_type, service_name)

    def call(
        self,
        request: SrvTypeRequest,
        timeout_sec: Optional[float] = None,
        max_wait_for_service_attempts: Optional[int] = None,
    ) -> Optional[SrvTypeResponse]:
        """Calls the service and returns the result. This is safe to call from a callback."""
        wait_for_service_attempts = 0
        while not self._client.wait_for_service(timeout_sec=timeout_sec):
            self._node_wrapper.get_logger().warn(f"{self._service_name} not available, waiting again...")
            wait_for_service_attempts += 1

            if max_wait_for_service_attempts is not None and wait_for_service_attempts >= max_wait_for_service_attempts:
                self._node_wrapper.get_logger().error(f"{self._service_name} not available!")
                return None

        future = self._client.call_async(request)
        if not self._node_wrapper.spin_until_future_complete(future):
            return None
        return future.result()

    def shutdown(self) -> None:
        self._node_wrapper.shutdown()
