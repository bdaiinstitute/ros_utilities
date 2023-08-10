# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import unittest
from threading import Thread
from typing import Optional

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy import Context
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from bdai_ros2_wrappers.service_client import ServiceClientWrapper


class MinimalService(Node):
    """
    Taken from Ros2 documentation on services with minor edits
    """

    def __init__(self, context: Context) -> None:
        super().__init__("minimal_service", context=context)
        self.srv = self.create_service(AddTwoInts, "add_two_ints", self.add_two_ints_callback)
        self._executor: MultiThreadedExecutor = MultiThreadedExecutor(num_threads=2, context=context)
        self._executor.add_node(self)
        self._thread: Optional[Thread] = Thread(target=self._spin)
        self._thread.start()

    def _spin(self) -> None:
        if self._executor is not None:
            try:
                self._executor.spin()
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
            self._executor.remove_node(self)
            self._executor = None
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self.srv = None

    def add_two_ints_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response) -> AddTwoInts.Response:
        response.sum = request.a + request.b
        self.get_logger().info("Incoming request\na: %d b: %d" % (request.a, request.b))

        return response


class ServiceClientWrapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context: Optional[Context] = Context()
        rclpy.init(context=self.context)
        self.service: Optional[MinimalService] = MinimalService(self.context)
        self.client_wrapper: ServiceClientWrapper = ServiceClientWrapper(
            "test_client", AddTwoInts, "add_two_ints", context=self.context
        )
        self.no_service_client: ServiceClientWrapper = ServiceClientWrapper(
            "test_client", AddTwoInts, "no_service", context=self.context
        )

    def tearDown(self) -> None:
        if self.service is not None:
            self.service.stop()
            self.service = None
        if self.client_wrapper is not None:
            self.client_wrapper = None
        if self.no_service_client is not None:
            self.no_service_client = None

        rclpy.shutdown(context=self.context)
        self.context = None

    def test_working_call(self) -> None:
        """
        Should test normal operation
        """
        req = AddTwoInts.Request()
        req.a = 10
        req.b = 23
        # send request
        result = self.client_wrapper.call(req)
        self.assertEqual(result.sum, 33)

    def test_timeout_no_service(self) -> None:
        """
        Should test timing out behavior
        """
        req = AddTwoInts.Request()
        req.a = 10
        req.b = 23
        # send request with client that has no service
        result = self.no_service_client.call(req, timeout_sec=0.5, max_wait_for_service_attempts=2)
        self.assertEqual(result, None)


if __name__ == "__main__":
    unittest.main()
