# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

from typing import Any, Optional, Type

from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.futures import wait_for_future


class ServiceException(Exception):
    """Base service exception."""

    def __init__(self, service: Future) -> None:
        super().__init__(service)
        self.service = service


class ServiceTimeout(ServiceException):
    """Exception raised on service timeout."""

    pass


class ServiceError(ServiceException):
    """Exception raised on service error."""

    pass


class Serviced:
    """An ergonomic interface to call services in ROS 2.

    Serviced instances wrap `rclpy.Client` instances to allow for synchronous
    and asynchronous service invocations, in a way that resembles remote
    procedure calls.
    """

    def __init__(self, service_type: Type, service_name: str, node: Optional[Node] = None, **kwargs: Any) -> None:
        node = node or scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._service_type = service_type
        self._service_name = service_name
        self._client = node.create_client(service_type, service_name, **kwargs)

    @property
    def service_name(self) -> str:
        """Get the target service name."""
        return self._service_name

    @property
    def service_type(self) -> Type:
        """Get the target service type."""
        return self._service_type

    @property
    def client(self) -> Client:
        """Get the underlying service client."""
        return self._client

    def wait_for_service(self, *args: Any, **kwargs: Any) -> bool:
        """Wait for service to become available.

        See `rclpy.Client.wait_for_service()` documentation for further reference.
        """
        return self._client.wait_for_service(*args, **kwargs)

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        """Forward invocation to `Serviced.synchronously`."""
        return self.synchronously(*args, **kwargs)

    def synchronously(
        self,
        request: Optional[Any] = None,
        *,
        timeout_sec: Optional[float] = None,
    ) -> Any:
        """Invoke service synchronously.

        Args:
            request: request to be serviced. If none is provided, a default initialized request
            will be used instead.
            timeout_sec: optional action timeout, in seconds. If a timeout is specified and it
            expires, the service request will be cancelled and the call will raise. Note this
            timeout is local to the caller.

        Returns:
            the service response.

        Raises:
            ServiceTimeout: if the service request timed out.
            ServiceError: if the service request failed.
        """
        future = self.asynchronously(request)
        if not wait_for_future(future, timeout_sec):
            future.cancel()
            raise ServiceTimeout(future)
        exception = future.exception()
        if exception is not None:
            raise ServiceError(future) from exception
        response = future.result()
        if not getattr(response, "success", True):
            raise ServiceError(future)
        return response

    def asynchronously(self, request: Optional[Any] = None) -> Future:
        """Invoke service asynchronously.

        Args:
            request: request to be serviced. If none is provided, a default
            initialized request will be used instead.

        Returns:
            the future response.
        """
        if request is None:
            request = self.service_type.Request()
        return self._client.call_async(request)
