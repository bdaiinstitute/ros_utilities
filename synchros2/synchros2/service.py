# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

from typing import Any, Generic, Optional, Protocol, Type, TypeVar, overload

from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future

import synchros2.scope as scope
from synchros2.callables import ComposableCallable, VectorizingCallable
from synchros2.futures import FutureLike, wait_for_future


class ServiceException(Exception):
    """Base service exception."""

    def __init__(self, service: Future) -> None:
        super().__init__(service)
        self.service = service

    def __str__(self) -> str:
        return "unknown service error"


class ServiceTimeout(ServiceException):
    """Exception raised on service timeout."""

    def __str__(self) -> str:
        return "service timed out"


class ServiceError(ServiceException):
    """Exception raised on service error."""

    def __str__(self) -> str:
        exception = self.service.exception()
        if exception is not None:
            return f"service failed (due to {exception})"
        result = self.service.result()
        if hasattr(result, "message"):
            return f"service failed (due to {result.message})"
        return "service failed"


ServiceRequestT = TypeVar("ServiceRequestT", contravariant=True)
ServiceResponseT = TypeVar("ServiceResponseT", covariant=True)


class ServicedProtocol(Protocol[ServiceRequestT, ServiceResponseT]):
    """Ergonomic protocol to call services in ROS 2."""

    @property
    def client(self) -> Client:
        """Get the underlying service client."""

    def wait_for_service(self, *args: Any, **kwargs: Any) -> bool:
        """Wait for service to become available."""

    @overload
    def synchronous(
        self,
        request: Optional[ServiceRequestT] = ...,
        *,
        timeout_sec: Optional[float] = ...,
    ) -> ServiceResponseT:
        """Invoke service synchronously.

        Args:
            request: request to be serviced, or a default initialized one if none is provided.
            timeout_sec: optional action timeout, in seconds. If a timeout is specified and it
            expires, the service request will be cancelled and the call will raise. Note this
            timeout is local to the caller.

        Returns:
            the service response.

        Raises:
            ServiceTimeout: if the service request timed out.
            ServiceError: if the service request failed.
        """

    @overload
    def synchronous(
        self,
        request: Optional[ServiceRequestT] = ...,
        *,
        timeout_sec: Optional[float] = ...,
        nothrow: bool = ...,
    ) -> Optional[ServiceResponseT]:
        """Invoke service synchronously.

        Args:
            request: request to be serviced, or a default initialized one if none is provided.
            timeout_sec: optional action timeout, in seconds. If a timeout is specified and it
            expires, the service request will be cancelled and the call will raise. Note this
            timeout is local to the caller.
            nothrow: when set, errors do not raise exceptions.

        Returns:
            the service response or None on timeout or failure.
        """

    def asynchronous(self, request: Optional[ServiceRequestT] = ...) -> FutureLike[ServiceResponseT]:
        """Invoke service asynchronously.

        Args:
            request: request to be serviced, or a default initialized one if none is provided.

        Returns:
            the future response.
        """


class Serviced(Generic[ServiceRequestT, ServiceResponseT], ComposableCallable, VectorizingCallable):
    """An ergonomic interface to call services in ROS 2.

    Serviced instances wrap `rclpy.Client` instances to allow for synchronous
    and asynchronous service invocations, in a way that resembles remote
    procedure calls.
    """

    def __init__(self, service_type: Type, service_name: str, node: Optional[Node] = None, **kwargs: Any) -> None:
        if node is None:
            node = scope.ensure_node()
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

    @overload
    def synchronous(
        self,
        request: Optional[ServiceRequestT] = ...,
        *,
        timeout_sec: Optional[float] = ...,
    ) -> ServiceResponseT:
        """Invoke service synchronously.

        Args:
            request: request to be serviced, or a default initialized one if none is provided.
            timeout_sec: optional action timeout, in seconds. If a timeout is specified and it
            expires, the service request will be cancelled and the call will raise. Note this
            timeout is local to the caller.

        Returns:
            the service response.

        Raises:
            ServiceTimeout: if the service request timed out.
            ServiceError: if the service request failed.
        """

    @overload
    def synchronous(
        self,
        request: Optional[ServiceRequestT] = ...,
        *,
        timeout_sec: Optional[float] = ...,
        nothrow: bool = ...,
    ) -> Optional[ServiceResponseT]:
        """Invoke service synchronously but never raise a exception.

        Args:
            request: request to be serviced, or a default initialized one if none is provided.
            timeout_sec: optional action timeout, in seconds. If a timeout is specified and it
            expires, the service request will be cancelled and the call will raise. Note this
            timeout is local to the caller.
            nothrow: when set, errors do not raise exceptions.

        Returns:
            the service response or None on timeout or error.
        """

    def synchronous(
        self,
        request: Optional[ServiceRequestT] = None,
        *,
        timeout_sec: Optional[float] = None,
        nothrow: bool = False,
    ) -> Optional[ServiceResponseT]:
        """Invoke service synchronously.

        Check available overloads documentation.
        """
        future = self.asynchronous(request)
        if not wait_for_future(future, timeout_sec):
            future.cancel()
            if nothrow:
                return None
            raise ServiceTimeout(future)
        exception = future.exception()
        if exception is not None:
            if nothrow:
                return None
            raise ServiceError(future) from exception
        response = future.result()
        if not nothrow and not getattr(response, "success", True):
            raise ServiceError(future)
        return response

    def asynchronous(self, request: Optional[ServiceRequestT] = None) -> FutureLike[ServiceResponseT]:
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
