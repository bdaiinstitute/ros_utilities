# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import functools
import threading
import typing


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
