# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
import threading
from typing import Any, Callable, Optional

from rclpy.context import Context
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.task import Future
from rclpy.utilities import get_default_context


class ServiceHandle:
    """A handle for the lifecycle of a service.

    Handles getting a result after sending an ServiceRequest to Service
    as holding the various callbacks for sending an ServiceRequest (result, failure)
    """

    def __init__(self, service_name: str, logger: Optional[RcutilsLogger] = None, context: Optional[Context] = None):
        """Constructor

        Args:
            service_name: The name of the service handle to be used in logging
            logger: An optional ros logger
            context: An optional ros context
        """
        if context is None:
            context = get_default_context()
        self._service_name = service_name
        self._send_service_future: Optional[Future] = None
        self._future_ready_event = threading.Event()
        context.on_shutdown(self._future_ready_event.set)
        self._result_callback: Optional[Callable] = None
        self._on_failure_callback: Optional[Callable] = None
        self._result: Optional[Any] = None
        if logger is None:
            self._logger = RcutilsLogger(name=f"{service_name} Handle")
        else:
            self._logger = logger

    @property
    def result(self) -> Optional[Any]:
        """Returns the result if one has been received from the service"""
        return self._result

    def set_result_callback(self, result_callback: Callable) -> None:
        """Sets the callback for when the future.result() completes."""
        self._result_callback = result_callback

    def set_send_service_future(self, send_service_future: Future) -> None:
        """Sets the future received from sending the Action.Goal"""
        self._send_service_future = send_service_future
        self._send_service_future.add_done_callback(self._service_result_callback)

    def set_on_failure_callback(self, on_failure_callback: Callable) -> None:
        """Set the callback to execute upon failure"""
        self._on_failure_callback = on_failure_callback

    def wait(self, timeout_sec: Optional[float] = None) -> bool:
        """Wait for service response, if any."""
        if self._send_service_future is None:
            raise RuntimeError("no future to wait on")
        return self._future_ready_event.wait(timeout_sec)

    def _service_result_callback(self, future: Future) -> None:
        """Callback that handles receiving a response from the Service"""
        result = future.result()

        if result is None:
            self._logger.error(f"Service request failed: {future.exception():!r}")
            self._failure()
            self._future_ready_event.set()
            return

        self._result = result

        if self._result_callback is not None:
            self._result_callback(result)

        # If there's a failure callback but no success case for result, warn and return
        service_has_success = hasattr(result, "success")

        if self._on_failure_callback is not None and not service_has_success:
            self._logger.warning("Failure callback is set, but result has no success attribute")
            self._future_ready_event.set()
            return

        if service_has_success:
            if not result.success:
                self._logger.error(f"Service failed; error is {result.message}")
                self._failure()
                self._future_ready_event.set()
                return
        else:
            self._logger.warning(f"Service {self._service_name} has no success or failure flag")

        self._logger.info("Service completed")
        self._future_ready_event.set()

    def _failure(self) -> None:
        """Triggers the internal failure callback if it exists"""
        if self._on_failure_callback is not None:
            self._on_failure_callback()
