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
import rclpy.node

from bdai_ros2_wrappers.executors import AutoScalingMultiThreadedExecutor
from bdai_ros2_wrappers.node import Node
from bdai_ros2_wrappers.utilities import either_or


class ROSAwareScope(typing.ContextManager["ROSAwareScope"]):
    """
    A context manager to enable ROS code in a given scope by providing
    an `rclpy` node and an autoscaling multi-threaded executor spinning
    in a background thread.
    """

    def __init__(
        self,
        name: str,
        context: typing.Optional[rclpy.context.Context] = None,
        node_args: typing.Optional[typing.Dict[str, typing.Any]] = None,
        executor_args: typing.Optional[typing.Dict[str, typing.Any]] = None,
    ):
        """
        Initializes the ROS aware scope.

        Args:
            name: name for this scope, and for the underlying node.
            context: optional context for the underlying node.
            node_args: optional keyword arguments for the underlying node.
            executor_args: optional keyword arguments for the underlying executor.
        """
        self._name = name
        self._context = context
        self._node: typing.Optional[rclpy.node.Node] = None
        self._node_args: typing.Dict[str, typing.Any] = node_args or {}
        self._executor: typing.Optional[rclpy.executors.Executor] = None
        self._executor_args: typing.Dict[str, typing.Any] = executor_args or {}
        self._background_thread: typing.Optional[threading.Thread] = None
        self._active = False

    def __enter__(self) -> "ROSAwareScope":
        """
        Activates scope.

        An active scope cannot be re-entered.
        """
        if self._active:
            raise RuntimeError("cannot enter active scope")
        self._node = Node(self._name, context=self._context, **self._node_args)
        try:
            self._executor = AutoScalingMultiThreadedExecutor(
                context=self._context, logger=self._node.get_logger(), **self._executor_args
            )
            self._background_thread = threading.Thread(target=self._executor.spin)
            try:
                self._executor.add_node(self._node)
                self._background_thread.start()
            except:
                self._executor.shutdown()
                self._executor = None
                self._background_thread = None
                raise
            self._active = True
            return self
        except:
            self._node.destroy_node()
            self._node = None
            raise

    def __exit__(self, *exc: typing.Any) -> None:
        """
        Deactivates scope.

        An inactive scope cannot be deactivated.
        """
        if not self._active:
            raise RuntimeError("cannot exit inactive scope")
        assert self._executor is not None
        assert self._background_thread is not None
        assert self._node is not None
        self._executor.shutdown()
        self._executor = None
        self._background_thread.join()
        self._background_thread = None
        self._node.destroy_node()
        self._node = None
        self._active = False

    @property
    def node(self) -> rclpy.node.Node:
        """Scope node, if active."""
        if not self._active:
            raise RuntimeError("scope is not active")
        assert self._node is not None
        return self._node

    @property
    def executor(self) -> rclpy.executors.Executor:
        """Scope executor, if active."""
        if not self._active:
            raise RuntimeError("scope is not active")
        assert self._executor is not None
        return self._executor


MainCallableTakingArgs = typing.Callable[[argparse.Namespace], typing.Optional[int]]
MainCallableTakingArgv = typing.Callable[[typing.Optional[typing.Sequence[str]]], typing.Optional[int]]
MainCallableTakingNoArgs = typing.Callable[[], typing.Optional[int]]
MainCallable = typing.Union[MainCallableTakingNoArgs, MainCallableTakingArgv, MainCallableTakingArgs]


class ROSAwareProcess:
    """
    A ``main``-like function wrapper that binds a ROS aware scope to each function invocation.

    Only one process can be invoked at a time, becoming the ``current`` process.
    This enables global access to the process and, in consequence, to its scope.

    This wrapper will rarely be used explicitly but through the `wrap` decorator.
    """

    lock = threading.Lock()

    current: typing.Optional["ROSAwareProcess"] = None

    def __init__(
        self,
        func: MainCallable,
        name: typing.Optional[str] = None,
        cli: typing.Optional[argparse.ArgumentParser] = None,
        init_args: typing.Optional[typing.Dict[str, typing.Any]] = None,
        node_args: typing.Optional[typing.Dict[str, typing.Any]] = None,
        executor_args: typing.Optional[typing.Dict[str, typing.Any]] = None,
    ):
        """
        Initializes the ROS aware process.

        Args:
            func: a ``main``-like function to wrap.
            name: an optional name for this process and the underlying node.
            If not provided, the basename without extension of the calling executable will be used.

        Additional keyword arguments will be forwarded to the process' `ROSAwareScope` instance.
        """
        self._func = func
        if name is None:
            if cli is None:
                program_name = os.path.basename(sys.argv[0])
            else:
                program_name = cli.prog
            name, _ = os.path.splitext(program_name)
        self._name = name
        self._cli = cli
        self._init_args = init_args or {}
        self._node_args = node_args or {}
        self._executor_args = executor_args or {}
        self._scope: typing.Optional[ROSAwareScope] = None
        functools.update_wrapper(self, self._func)

    def __getattr__(self, name: str) -> typing.Any:
        """Gets attribute from scope, if any."""
        if self._scope is None:
            return super().__getattribute__(name)
        return getattr(self._scope, name)

    def __call__(self, argv: typing.Optional[typing.Sequence[str]] = None) -> typing.Optional[int]:
        """
        Invokes wrapped function in a ROS aware scope.

        Args:
            argv: optional command line-like arguments.

        Returns:
            an optional return code.

        Raises:
            RuntimeError: if a ROS aware process is already running.
        """
        if not ROSAwareProcess.lock.acquire(blocking=False):
            raise RuntimeError("Process already running!")

        args = None
        if self._cli is not None:
            args = self._cli.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])

        try:
            rclpy.init(args=argv, **either_or(args, "init_args", self._init_args))
            try:
                context = rclpy.get_default_context()
                node_args = either_or(args, "node_args", self._node_args)
                executor_args = either_or(args, "executor_args", self._executor_args)
                with ROSAwareScope(self._name, context, node_args, executor_args) as scope:
                    self._scope = scope
                    ROSAwareProcess.current = self
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
                        ROSAwareProcess.current = None
                        self._scope = None
            finally:
                rclpy.try_shutdown()
        finally:
            ROSAwareProcess.lock.release()


def node() -> typing.Optional[rclpy.node.Node]:
    """Gets the node of the current ROS aware process, if any."""
    process = ROSAwareProcess.current
    if process is None:
        return None
    return process.node


def executor() -> typing.Optional[rclpy.executors.Executor]:
    """Gets the executor of the current ROS aware process, if any."""
    process = ROSAwareProcess.current
    if process is None:
        return None
    return process.executor


def main(
    cli: typing.Optional[argparse.ArgumentParser] = None, **kwargs: typing.Any
) -> typing.Callable[[MainCallable], ROSAwareProcess]:
    """Wraps a ``main``-like function in a `ROSAwareProcess` instance."""

    def __decorator(func: MainCallable) -> ROSAwareProcess:
        return ROSAwareProcess(func, cli=cli, **kwargs)

    return __decorator
