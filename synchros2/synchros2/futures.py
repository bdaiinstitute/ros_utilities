# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
from threading import Event
from typing import Any, Awaitable, Callable, Optional, Protocol, TypeVar, Union, runtime_checkable

from rclpy.clock import Clock
from rclpy.context import Context
from rclpy.utilities import get_default_context

from synchros2.clock import wait_for

T = TypeVar("T", covariant=True)


class FutureLike(Awaitable[T], Protocol[T]):
    """A future-like awaitable object.

    Matches `rclpy.task.Future` and `concurrent.futures.Future` protocols.
    """

    def result(self) -> T:
        """Get future result (may block)."""
        ...

    def exception(self) -> Optional[Exception]:
        """Get future exception, if any."""
        ...

    def done(self) -> bool:
        """Check if future is ready."""
        ...

    def add_done_callback(self, func: Callable[["FutureLike[T]"], None]) -> None:
        """Add a callback to be scheduled as soon as the future is ready."""
        ...

    def cancel(self) -> None:
        """Cancel future."""
        ...

    def cancelled(self) -> bool:
        """Check if future was cancelled."""
        ...


@runtime_checkable
class FutureConvertible(Awaitable[T], Protocol[T]):
    """An awaitable that is convertible to a future-like object."""

    def as_future(self) -> FutureLike[T]:
        """Get future-like view."""
        ...


AnyFuture = Union[FutureLike, FutureConvertible]


def as_proper_future(instance: AnyFuture) -> FutureLike:
    """Return `instance` as a proper future-like object."""
    if isinstance(instance, FutureConvertible):
        return instance.as_future()
    return instance


def wait_for_future(
    future: AnyFuture,
    timeout_sec: Optional[float] = None,
    *,
    clock: Optional[Clock] = None,
    context: Optional[Context] = None,
) -> bool:
    """Block while waiting for a future to become done

    Args:
        future (Future): The future to be waited on
        timeout_sec (Optional[float]): An optional timeout for how long to wait
        clock (Optional[Clock]): An optional clock to use for timeout waits,
        defaults to the clock of the current scope if any, otherwise the system clock
        context (Optional[Context]): Current context (will use the default if none is given)

    Returns:
        bool: True if successful, False if the timeout was triggered
    """
    if context is None:
        context = get_default_context()
    if clock is None:
        import synchros2.scope

        clock = synchros2.scope.clock()
    event = Event()
    context.on_shutdown(event.set)
    proper_future = as_proper_future(future)
    proper_future.add_done_callback(lambda _: event.set())
    if proper_future.cancelled():
        event.set()
    wait_for(event, clock=clock, timeout_sec=timeout_sec)
    return proper_future.done()


def unwrap_future(
    future: AnyFuture,
    timeout_sec: Optional[float] = None,
    *,
    clock: Optional[Clock] = None,
    context: Optional[Context] = None,
) -> Any:
    """Fetch future result when it is done.

    Note this function may block and may raise if the future does or it times out
    waiting for it. See wait_for_future() documentation for further reference on
    arguments taken.
    """
    proper_future = as_proper_future(future)
    if not wait_for_future(proper_future, timeout_sec, clock=clock, context=context):
        raise ValueError("cannot unwrap future that is not done")
    return proper_future.result()
