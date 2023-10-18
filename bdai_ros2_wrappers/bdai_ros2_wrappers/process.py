# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import argparse
import functools
import inspect
import os
import sys
import threading
import typing

import rclpy
import rclpy.executors
import rclpy.logging
import rclpy.node

import bdai_ros2_wrappers.context as context
import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.scope import AnyEntity, AnyEntityFactoryCallable, ROSAwareScope
from bdai_ros2_wrappers.utilities import either_or

MainCallableTakingArgs = typing.Callable[[argparse.Namespace], typing.Optional[int]]
MainCallableTakingArgv = typing.Callable[[typing.Sequence[str]], typing.Optional[int]]
MainCallableTakingNoArgs = typing.Callable[[], typing.Optional[int]]
MainCallable = typing.Union[MainCallableTakingNoArgs, MainCallableTakingArgv, MainCallableTakingArgs]


class ROSAwareProcess:
    """
    A ``main``-like function wrapper that binds a ROS 2 aware scope to each function invocation.

    Only one process can be invoked at a time, becoming the ``current`` process.
    This enables global access to the process and, in consequence, to its scope.

    This wrapper is rarely be used explicitly but through the `main` decorator.
    """

    lock = threading.Lock()

    current: typing.Optional["ROSAwareProcess"] = None

    def __init__(
        self,
        func: MainCallable,
        *,
        prebaked: bool = True,
        autospin: typing.Optional[bool] = None,
        forward_logging: typing.Optional[bool] = None,
        namespace: typing.Optional[typing.Union[typing.Literal[True], str]] = None,
        cli: typing.Optional[argparse.ArgumentParser] = None,
        **init_arguments: typing.Any,
    ):
        """
        Initializes the ROS 2 aware process.

        Args:
            func: a ``main``-like function to wrap ie. a callable taking a sequence of strings,
            an `argparse.Namespace` (if a CLI is specified), or nothing, and returning a integer
            exit code or nothing.
            prebaked: whether to instantiate a prebaked or a bare process. Bare processes do not bear
            a node nor spin an executor, which brings them closest to standard ROS 2 idioms.
            autospin: whether to automatically equip the underlying scope with a background executor
            or not. Defaults to True for prebaked processes and to False for bare processes.
            forward_logging: whether to forward `logging` logs to the ROS 2 logging system or not.
            Defaults to True for prebaked processes and to False for bare processes (though it requires
            a process node to be set to function).
            namespace: an optional namespace for this process. If True, the current executable basename
            without its extension (or CLI program name if one is specified) will be used. Defaults to
            True for prebaked processes.
            cli: optional command-line interface argument parser.

        Raises:
            ValueError: if a prebaked process is configured without autospin.
        """
        if forward_logging is None:
            forward_logging = prebaked
        if autospin is None:
            autospin = prebaked
        if namespace is None and prebaked:
            namespace = True
        if namespace is True:
            if cli is None:
                program_name = os.path.basename(sys.argv[0])
            else:
                program_name = cli.prog
            namespace, _ = os.path.splitext(program_name)
        self._func = func
        self._cli = cli
        self._scope_kwargs = dict(
            prebaked=prebaked, autospin=autospin, namespace=namespace, forward_logging=forward_logging
        )
        self._scope: typing.Optional[ROSAwareScope] = None
        self._lock = threading.Lock()
        functools.update_wrapper(self, self._func)

    def __getattr__(self, name: str) -> typing.Any:
        """
        Gets missing attributes from the underlying scope.

        Raises:
            RuntimeError: if the process is not executing.
        """
        with self._lock:
            if self._scope is None:
                raise RuntimeError("process is not executing")
            return getattr(self._scope, name)

    def __setattr__(self, name: str, value: typing.Any) -> None:
        """
        Sets public attributes on the underlying scope when possible.

        Args:
            name: name of the attribute to be set
            value: attribute value to be set.
        """
        if not name.startswith("_") and hasattr(self._scope, name):
            setattr(self._scope, name, value)
            return
        super().__setattr__(name, value)

    def __call__(self, argv: typing.Optional[typing.Sequence[str]] = None) -> typing.Optional[int]:
        """
        Invokes wrapped process function in a ROS 2 aware scope.

        Args:
            argv: optional command line-like arguments.

        Returns:
            an optional return code.

        Raises:
            RuntimeError: if a ROS 2 aware process is already running.
        """
        if not ROSAwareProcess.lock.acquire(blocking=False):
            raise RuntimeError("process already running")
        self._lock.acquire()
        try:
            args = None
            if self._cli is not None:
                args = self._cli.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])
            scope_kwargs = either_or(args, "process_args", self._scope_kwargs)
            with scope.top(argv, global_=True, **scope_kwargs) as self._scope:
                ROSAwareProcess.current = self
                self._lock.release()
                try:
                    sig = inspect.signature(self._func)
                    if not sig.parameters:
                        func_taking_no_args = typing.cast(MainCallableTakingNoArgs, self._func)
                        return func_taking_no_args()
                    if args is not None:
                        func_taking_args = typing.cast(MainCallableTakingArgs, self._func)
                        return func_taking_args(args)
                    func_taking_argv = typing.cast(MainCallableTakingArgv, self._func)
                    return func_taking_argv(rclpy.utilities.remove_ros_args(argv))
                finally:
                    self._lock.acquire()
                    ROSAwareProcess.current = None
                    self._scope = None
        except KeyboardInterrupt:
            return None  # ignore
        finally:
            self._lock.release()
            ROSAwareProcess.lock.release()

    def wait_for_shutdown(self, *, timeout_sec: typing.Optional[float] = None) -> bool:
        """
        Wait for shutdown of the underlying scope context.

        Args:
            timeout_sec: optional timeout for wait, wait indefinitely by default.

        Returns:
            True if shutdown, False on timeout.
        """
        with self._lock:
            if self._scope is None:
                raise RuntimeError("process is not executing")
            return context.wait_for_shutdown(timeout_sec=timeout_sec, context=self._scope.context)

    def try_shutdown(self) -> None:
        """Atempts to shutdown the underlying scope context."""
        with self._lock:
            if self._scope is None:
                raise RuntimeError("process is not executing")
            rclpy.try_shutdown(context=self._scope.context)


def current() -> typing.Optional[ROSAwareProcess]:
    """Gets the current ROS 2 aware process, if any."""
    return ROSAwareProcess.current


def node() -> typing.Optional[rclpy.node.Node]:
    """Gets the node of the current ROS 2 aware process, if any."""
    process = current()
    if process is None:
        return None
    return process.node


def executor() -> typing.Optional[rclpy.executors.Executor]:
    """Gets the executor of the current ROS 2 aware process, if any."""
    process = current()
    if process is None:
        return None
    return process.executor


def load(factory: AnyEntityFactoryCallable, *args: typing.Any, **kwargs: typing.Any) -> AnyEntity:
    """
    Loads a ROS 2 node (or a collection thereof) within the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.load` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    return process.load(factory, *args, **kwargs)


def unload(loaded: AnyEntity) -> None:
    """
    Unloads a ROS 2 node (or a collection thereof) from the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.unload` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    process.unload(loaded)


def managed(
    factory: AnyEntityFactoryCallable, *args: typing.Any, **kwargs: typing.Any
) -> typing.ContextManager[AnyEntity]:
    """
    Manages a ROS 2 node (or a collection thereof) within the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.managed` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    return process.managed(factory, *args, **kwargs)


def spin(factory: typing.Optional[AnyEntityFactoryCallable] = None, *args: typing.Any, **kwargs: typing.Any) -> None:
    """
    Spins current ROS 2 aware process executor (and all ROS 2 nodes in it).

    Optionally, manages a ROS 2 node (or a collection thereof) for as long as it spins.

    See `ROSAwareProcess` and `ROSAwareScope.spin` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    process.spin(factory, *args, **kwargs)


def main(
    cli: typing.Optional[argparse.ArgumentParser] = None, **kwargs: typing.Any
) -> typing.Callable[[MainCallable], ROSAwareProcess]:
    """Wraps a ``main``-like function in a `ROSAwareProcess` instance."""

    def __decorator(func: MainCallable) -> ROSAwareProcess:
        return ROSAwareProcess(func, cli=cli, **kwargs)

    return __decorator


def try_shutdown() -> None:
    """Attempts to shutdown the context associated with the current ROS 2 aware process."""
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    process.try_shutdown()


def wait_for_shutdown(*, timeout_sec: typing.Optional[float] = None) -> bool:
    """
    Wait for current ROS 2 aware process to shutdown.

    See `ROSAwareProcess.wait_for_shutdown` documentation for further reference
    on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    return process.wait_for_shutdown(timeout_sec=timeout_sec)
