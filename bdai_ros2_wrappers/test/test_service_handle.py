#  Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
import unittest
from threading import Thread
from typing import Callable, Optional, Tuple, Union

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool, Trigger

from bdai_ros2_wrappers.service_handle import ServiceHandle
from bdai_ros2_wrappers.type_hints import Srv, SrvTypeRequest, SrvTypeResponse


def _callback_trigger_success(req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
    result = Trigger.Response()
    result.success = True
    result.message = "foo"
    return result


def _callback_trigger_fail(req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
    result = Trigger.Response()
    result.success = False
    result.message = "bar"
    return result


def _callback_setbool_response(req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
    result = SetBool.Response()
    if req.data:
        result.success = True
        result.message = "foo"
    else:
        result.success = False
        result.message = "bar"
    return result


def _callback_empty_service(req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
    result = Empty.Response()
    return result


class FooBarService(Node):
    """Service that returns a foo or bar message depending on the callback."""

    def __init__(self, service_type: Srv, service_name: str, callback: Callable) -> None:
        super().__init__("foobar_server")
        self._service = self.create_service(service_type, service_name, callback=callback)
        self._executor: Optional[MultiThreadedExecutor] = MultiThreadedExecutor(3)
        self._executor.add_node(self)
        self._keep_spinning = False
        self._thread: Optional[Thread] = Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        if self._executor is not None:
            self._keep_spinning = True
            try:
                while self._context.ok() and self._keep_spinning:
                    self._executor.spin_once(timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                pass
            self._keep_spinning = False

    def stop(self) -> None:
        self._keep_spinning = False
        if self._executor is not None:
            self._executor.shutdown()
            self._executor.remove_node(self)
            self._executor = None
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self._service = None


class ServiceHandleBaseTest(unittest.TestCase):
    """Base class that handles the setup for each test case."""

    def _setUp(self, node_name: str, service_name: str, service_type: Srv) -> None:
        rclpy.init()
        self.node_name = node_name
        self.service_name = service_name
        self.service_type = service_type
        self.client_node: Optional[Node] = Node(self.node_name)
        self.client = self.client_node.create_client(
            self.service_type, self.service_name, callback_group=ReentrantCallbackGroup()
        )
        self.server_node: Optional[FooBarService] = None

    def tearDown(self) -> None:
        if self.client_node is not None:
            self.client_node.destroy_node()
            self.client_node = None
        if self.server_node is not None:
            self.server_node.stop()
            self.server_node.destroy_node()
        self.server_node = None
        rclpy.shutdown()

    def _internal(self, *args: Union[str, bool]) -> Tuple[SrvTypeResponse, bool]:
        self.assertTrue(self.client.wait_for_service(1))
        failure = False

        def _failure_callback() -> None:
            nonlocal failure
            failure = True

        handle = ServiceHandle(args[0])
        handle.set_on_failure_callback(_failure_callback)

        req = self.service_type.Request()
        service_future = self.client.call_async(req)
        handle.set_send_service_future(service_future)

        while handle.result is None:
            rclpy.spin_once(
                self.client_node,
            )
        return handle.result, failure


class ServiceHandleTriggerTest(ServiceHandleBaseTest):
    """Checks if the Trigger Request is properly processed."""

    def setUp(self) -> None:
        super()._setUp(node_name="TriggerTest", service_name="trigger_service", service_type=Trigger)

    def test_success(self) -> None:
        """ "Request a successful trigger. Should return a success condition with the message foo."""
        self.server_node = FooBarService(self.service_type, self.service_name, callback=_callback_trigger_success)
        result, failure = self._internal("trigger_success")
        self.assertTrue(result.success)
        self.assertFalse(failure)
        self.assertEqual(result.message, "foo")

    def test_failure(self) -> None:
        """Request a failed trigger. Should return a success condition with the message bar."""
        self.server_node = FooBarService(self.service_type, self.service_name, callback=_callback_trigger_fail)
        result, failure = self._internal("trigger_failure")
        self.assertFalse(result.success)
        self.assertTrue(failure)


class ServiceHandleSetBoolTest(ServiceHandleBaseTest):
    """Checks if the SetBool Request is properly processed."""

    def setUp(self) -> None:
        super()._setUp(node_name="SetBoolTest", service_name="setbool_service", service_type=SetBool)

    def _internal(self, *args: Union[str, bool]) -> Tuple[SrvTypeResponse, bool]:
        self.assertTrue(self.client.wait_for_service(1))
        failure = False

        def _failure_callback() -> None:
            nonlocal failure
            failure = True

        handle = ServiceHandle(args[0])
        handle.set_on_failure_callback(_failure_callback)

        req = self.service_type.Request()
        req.data = args[1]
        service_future = self.client.call_async(req)
        handle.set_send_service_future(service_future)

        while handle.result is None:
            rclpy.spin_once(
                self.client_node,
            )
        return handle.result, failure

    def test_success(self) -> None:
        """ "Request data passes a true value. Should return a success condition with the message foo."""
        self.server_node = FooBarService(self.service_type, self.service_name, callback=_callback_setbool_response)
        self.assertIsNotNone(self.server_node)
        result, failure = self._internal("setbool_success", True)
        self.assertTrue(result.success)
        self.assertEqual(result.message, "foo")
        self.assertFalse(failure)

    def test_reject(self) -> None:
        """ "Request data passes a false value. Should return a failure condition with the message bar."""
        self.server_node = FooBarService(self.service_type, self.service_name, callback=_callback_setbool_response)
        result, failure = self._internal("setbool_reject", False)
        self.assertTrue(failure)
        self.assertFalse(result.success)
        self.assertEqual(result.message, "bar")


class ServiceHandleEmptyTest(ServiceHandleBaseTest):
    """Checks if the Empty Request is properly processed."""

    def setUp(self) -> None:
        super()._setUp(node_name="EmptyTest", service_name="empty_service", service_type=Empty)

    def test_warning(self) -> None:
        """Tests that the result callback should give a warning and have no fields."""
        self.server_node = FooBarService(self.service_type, self.service_name, callback=_callback_empty_service)
        result, failure = self._internal("empty_test_result")
        self.assertIsNotNone(result)
        self.assertFalse(hasattr(result, "success"))
        self.assertFalse(hasattr(result, "message"))


if __name__ == "__main__":
    unittest.main()
