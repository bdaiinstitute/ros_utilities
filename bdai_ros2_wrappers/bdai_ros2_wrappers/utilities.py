# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import functools
import threading
import typing

import rclpy.clock
import rclpy.duration
import rclpy.time


def namespace_with(*args: typing.Optional[str]) -> str:
    """Puts together a ROS 2 like namespace from its constitutive parts."""
    sanitized_args = list(filter(None, args))
    if not sanitized_args:
        raise ValueError("nothing to namespace")
    sanitized_args = [arg.rstrip("/") for arg in sanitized_args]
    namespace = sanitized_args[0]
    for arg in sanitized_args[1:]:
        if not arg.startswith("/"):
            namespace += "/" + arg
        else:
            namespace = arg
    return namespace


def either_or(obj: typing.Any, name: str, default: typing.Any) -> typing.Any:
    """Gets either an object attribute's value or a default value.

    Unlike `getattr`, callable attributes are applied as getters on `obj`.
    """
    if not hasattr(obj, name):
        return default
    value_or_getter = getattr(obj, name)
    if callable(value_or_getter):
        return value_or_getter(obj)
    return value_or_getter


def fqn(obj: typing.Any) -> typing.Optional[str]:
    """Computes the fully qualified name of a given object, if any."""
    if not hasattr(obj, "__qualname__"):
        return None
    name = obj.__qualname__
    if not hasattr(obj, "__module__"):
        return name
    module = obj.__module__
    return f"{module}.{name}"


def bind_to_thread(callable_: typing.Callable, thread: threading.Thread) -> typing.Callable:
    """Binds a callable to a thread, so it can only be invoked from that thread."""

    @functools.wraps(callable_)
    def _wrapper(*args: typing.Any, **kwargs: typing.Any) -> typing.Any:
        if threading.current_thread() is not thread:
            raise RuntimeError(f"{fqn(callable_)}() is bound to a different thread")
        return callable_(*args, **kwargs)

    return _wrapper


def synchronized(
    func: typing.Optional[typing.Callable] = None,
    lock: typing.Optional[threading.Lock] = None,
) -> typing.Callable:
    """Wraps `func` to synchronize invocations, optionally taking a user defined `lock`.

    This function can be used as a decorator, like:

    @synchronized
    def my_function(...):
        ...

    or

    @synchronized(lock=my_lock)
    def my_function(...):
        ...
    """
    if lock is None:
        lock = threading.Lock()
    assert lock is not None

    def _decorator(func: typing.Callable) -> typing.Callable:
        @functools.wraps(func)
        def __wrapper(*args: typing.Any, **kwargs: typing.Any) -> typing.Any:
            with lock:  # type: ignore
                return func(*args, **kwargs)

        return __wrapper

    if func is None:
        return _decorator
    return _decorator(func)


def functional_decorator(base_decorator: typing.Callable) -> typing.Callable:
    """Wraps a decorating callable to be usable as a Python decorator for functions.

    As an example, consider the following decorator example:

    @functional_decorator
    def my_decorator(func, some_flag=None):
        ...

    This decorator can then be used like this:

    @my_decorator
    def my_function(*args):
        ...

    and also like this:

    @my_decorator(some_flag=True)
    def my_function(*args):
        ...
    """

    @functools.wraps(base_decorator)
    def _wrapper(func: typing.Optional[typing.Callable] = None, **kwargs: typing.Any) -> typing.Callable:
        def _bound_decorator(func: typing.Callable) -> typing.Callable:
            return base_decorator(func, **kwargs)

        if func is None:
            return _bound_decorator
        return _bound_decorator(func)

    return _wrapper


@functional_decorator
def throttle(
    func: typing.Callable,
    min_period: rclpy.duration.Duration,
    time_source: typing.Optional[rclpy.clock.Clock] = None,
    fill_value: typing.Any = None,
) -> typing.Callable:
    """Decorates a callable to throttle invocations.

    Args:
        func: callable to be decorated.
        min_period: minimum time between consecutive invocations.
        time_source: optional time source to measure time against.
        If none is provided, the system clock will be used.
        fill_value: optional value to return for throttled invocations.

    Returns:
        decorated callable.
    """
    safe_time_source = rclpy.clock.Clock() if time_source is None else time_source
    time_of_last_call: typing.Optional[rclpy.time.Time] = None

    @functools.wraps(func)
    def _wrapper(*args: typing.Any, **kwargs: typing.Any) -> typing.Any:
        nonlocal time_of_last_call
        return_value = fill_value
        current_time = safe_time_source.now()
        if time_of_last_call is None or current_time - time_of_last_call >= min_period:
            return_value = func(*args, **kwargs)
            time_of_last_call = current_time
        return return_value

    return _wrapper


@functional_decorator
def skip(func: typing.Callable, num_times: int, fill_value: typing.Any = None) -> typing.Callable:
    """Decorates a callable to skip the first few invocations a prescribed number of times.

    Args:
        func: callable to be decorated.
        num_times: number of times to skip the invocation.
        fill_value: optional value to return for skipped invocations.

    Returns:
        decorated callable.
    """
    num_skipped_calls = 0

    @functools.wraps(func)
    def _wrapper(*args: typing.Any, **kwargs: typing.Any) -> typing.Any:
        nonlocal num_skipped_calls
        if num_skipped_calls < num_times:
            num_skipped_calls += 1
            return fill_value
        return func(*args, **kwargs)

    return _wrapper


@functional_decorator
def cap(func: typing.Callable, num_times: int, fill_value: typing.Any = None) -> typing.Callable:
    """Decorates a callable to cap invocations to a prescribed number of times.

    Args:
        func: callable to be decorated.
        num_times: maximum number of times the callable may be invoked.
        fill_value: optional value to return once invocations reach their cap.

    Returns:
        decorated callable.
    """
    num_calls = 0

    @functools.wraps(func)
    def _wrapper(*args: typing.Any, **kwargs: typing.Any) -> typing.Any:
        nonlocal num_calls
        if num_calls < num_times:
            num_calls += 1
            return func(*args, **kwargs)
        return fill_value

    return _wrapper
