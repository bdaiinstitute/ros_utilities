# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
import argparse
import functools
import inspect
import os
import signal
import sys
import threading
import typing
import warnings

import rclpy
import rclpy.executors
import rclpy.logging
import rclpy.node
import rclpy.utilities
from rclpy.exceptions import InvalidNamespaceException, InvalidNodeNameException
from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

import synchros2.context as context
import synchros2.scope as scope
from synchros2.scope import ROSAwareScope
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import either_or

NodeT = typing.TypeVar("NodeT", bound=rclpy.node.Node)
NodeFactoryCallable = typing.Callable[..., NodeT]
GraphFactoryCallable = typing.Callable[..., typing.List[NodeT]]

MainCallableTakingArgs = typing.Callable[[argparse.Namespace], typing.Optional[int]]
MainCallableTakingArgv = typing.Callable[[typing.Sequence[str]], typing.Optional[int]]
MainCallableTakingNoArgs = typing.Callable[[], typing.Optional[int]]
MainCallable = typing.Union[MainCallableTakingNoArgs, MainCallableTakingArgv, MainCallableTakingArgs]


class ROSAwareProcess:
    """A ``main``-like function wrapper that binds a ROS 2 aware scope to each function invocation.

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
        uses_tf: bool = False,
        interruptible: bool = False,
        prebaked: typing.Union[bool, str] = True,
        autospin: typing.Optional[bool] = None,
        forward_logging: typing.Optional[bool] = None,
        namespace: typing.Optional[typing.Union[typing.Literal[True], str]] = None,
        cli: typing.Optional[argparse.ArgumentParser] = None,
        **init_arguments: typing.Any,
    ):
        """Initializes the ROS 2 aware process.

        Args:
            func: a ``main``-like function to wrap ie. a callable taking a sequence of strings,
            an `argparse.Namespace` (if a CLI is specified), or nothing, and returning a integer
            exit code or nothing.
            prebaked: whether to instantiate a prebaked or a bare process i.e. a process bearing
            an implicit node and executor, or not. May also specificy the exact name for the implicit
            node, or, if True, the current executable basename without its extension (or CLI program
            name if one is specified) will be used. Note that completely bare processes do not bear a
            node nor spin an executor, which brings them closest to standard ROS 2 idioms.
            autospin: whether to automatically equip the underlying scope with a background executor
            or not. Defaults to True for prebaked processes and to False for bare processes.
            uses_tf: whether to instantiate a tf listener bound to the process main node. Defaults to False.
            interruptible: whether the process allows graceful interruptions i.e. SIGINT (Ctrl+C) or SIGTERM
            signaling. An interruptible process will not shutdown any context or trigger any guard condition
            on either but simply raise KeyboardInterrupt and SystemExit exceptions instead.
            forward_logging: whether to forward `logging` logs to the ROS 2 logging system or not.
            Defaults to True for prebaked processes and to False for bare processes (though it requires
            a process node to be set to function).
            namespace: an optional namespace for this process. If True, the current executable basename
            without its extension (or CLI program name if one is specified) will be used. Defaults to
            True for prebaked processes.
            cli: optional command-line interface argument parser.
            init_arguments: Keyword arguments for the scope.

        Raises:
            ValueError: if a prebaked process is configured without autospin.
        """
        program_name = os.path.basename(sys.argv[0]) if cli is None else cli.prog
        name, _ = os.path.splitext(program_name)
        if prebaked is True:
            prebaked = name
        if namespace is True:
            namespace = name
        if forward_logging is None:
            forward_logging = bool(prebaked)
        self._lock = threading.Lock()
        self._scope: typing.Optional[ROSAwareScope] = None
        self._func = func
        self._cli = cli
        self._interruptible = interruptible
        self._scope_kwargs = dict(
            prebaked=prebaked,
            autospin=autospin,
            uses_tf=uses_tf,
            namespace=namespace,
            forward_logging=forward_logging,
            **init_arguments,
        )
        functools.update_wrapper(self, self._func)

    @property
    def cli(self) -> typing.Optional[argparse.ArgumentParser]:
        """Get the associated CLI argument parser, if any."""
        return self._cli

    def __getattr__(self, name: str) -> typing.Any:
        """Gets missing attributes from the underlying scope.

        Raises:
            RuntimeError: if the process is not executing.
        """
        with self._lock:
            if self._scope is None:
                raise RuntimeError("process is not executing")
            return getattr(self._scope, name)

    def __setattr__(self, name: str, value: typing.Any) -> None:
        """Sets public attributes on the underlying scope when possible.

        Args:
            name: name of the attribute to be set
            value: attribute value to be set.
        """
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("process attributes can only be set from the main thread")
        if not name.startswith("_") and hasattr(self._scope, name):
            setattr(self._scope, name, value)
            return
        super().__setattr__(name, value)

    def __call__(self, argv: typing.Optional[typing.Sequence[str]] = None) -> typing.Optional[int]:
        """Invokes wrapped process function in a ROS 2 aware scope.

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
            scope_kwargs = dict(self._scope_kwargs)
            if self._cli is not None:
                args = self._cli.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])
                scope_kwargs.update(either_or(args, "process_args", {}))
            prebaked = scope_kwargs.get("prebaked")
            if isinstance(prebaked, str):
                if args is not None:
                    prebaked = prebaked.format(**vars(args))
                try:
                    validate_node_name(prebaked)
                except InvalidNodeNameException:
                    warnings.warn(f"'{prebaked}' cannot be used as node name, using scope default", stacklevel=1)
                    prebaked = True
                scope_kwargs["prebaked"] = prebaked
            namespace = scope_kwargs.get("namespace")
            if isinstance(namespace, str):
                if args is not None:
                    namespace = namespace.format(**vars(args))
                try:
                    validate_namespace(namespace)
                except InvalidNamespaceException:
                    warnings.warn(f"'{namespace}' cannot be used as namespace, using scope default", stacklevel=1)
                    namespace = True
                scope_kwargs["namespace"] = namespace
            with scope.top(argv, global_=True, interruptible=self._interruptible, **scope_kwargs) as self._scope:
                ROSAwareProcess.current = self
                self._lock.release()
                orig_sigterm_handler: typing.Optional[typing.Any] = None
                if self._interruptible:

                    def handle_sigterm(*args: typing.Any) -> None:
                        nonlocal orig_sigterm_handler
                        if orig_sigterm_handler is not None and orig_sigterm_handler != signal.SIG_DFL:
                            orig_sigterm_handler(*args)
                        raise SystemExit("due to SIGTERM")

                    orig_sigterm_handler = signal.signal(signal.SIGTERM, handle_sigterm)
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
                    if orig_sigterm_handler is not None:
                        signal.signal(signal.SIGTERM, orig_sigterm_handler)
                    self._lock.acquire()
                    ROSAwareProcess.current = None
                    self._scope = None
        except KeyboardInterrupt:
            return None  # ignore
        finally:
            self._lock.release()
            ROSAwareProcess.lock.release()

    def wait_for_interrupt(self, *, timeout_sec: typing.Optional[float] = None) -> None:
        """Wait for process interruption i.e. a KeyboardInterrupt or a SystemExit.

        This can only be done from the main thread. Also, note that timeouts are
        implemented using POSIX timers and alarms.

        Args:
            timeout_sec: optional timeout for wait, wait indefinitely by default.
        """
        if not self._interruptible:
            raise RuntimeError("process is not interruptible")
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("interrupts can only be awaited from the main thread")
        if self._scope is None:
            raise RuntimeError("process is not executing")
        interrupted = False
        orig_sigalrm_handler = None
        orig_itimer_setup = None
        if timeout_sec is not None:

            def handle_sigalarm(*args: typing.Any) -> None:
                nonlocal interrupted
                interrupted = True

            orig_sigalrm_handler = signal.signal(signal.SIGALRM, handle_sigalarm)
            orig_itimer_setup = signal.setitimer(signal.ITIMER_REAL, timeout_sec)
        try:
            while not interrupted:
                signal.pause()
        finally:
            if orig_itimer_setup is not None:
                signal.setitimer(signal.ITIMER_REAL, *orig_itimer_setup)
            if orig_sigalrm_handler is not None:
                signal.signal(signal.SIGALRM, orig_sigalrm_handler)

    def wait_for_shutdown(self, *, timeout_sec: typing.Optional[float] = None) -> bool:
        """Wait for shutdown of the underlying scope context.

        Args:
            timeout_sec: optional timeout for wait, wait indefinitely by default.

        Returns:
            True if shutdown, False on timeout.
        """
        return context.wait_for_shutdown(timeout_sec=timeout_sec, context=self.context)

    def try_shutdown(self) -> None:
        """Atempts to shutdown the underlying scope context."""
        rclpy.utilities.try_shutdown(context=self.context)


def current() -> typing.Optional[ROSAwareProcess]:
    """Gets the current ROS 2 aware process, if any."""
    return ROSAwareProcess.current


def node() -> typing.Optional[rclpy.node.Node]:
    """Gets the node of the current ROS 2 aware process, if any."""
    process = current()
    if process is None:
        return None
    return process.node


def ensure_node() -> rclpy.node.Node:
    """Gets a node from the current ROS 2 aware process or fails trying"""
    current_node = node()
    if current_node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    return current_node


def executor() -> typing.Optional[rclpy.executors.Executor]:
    """Gets the executor of the current ROS 2 aware process, if any."""
    process = current()
    if process is None:
        return None
    return process.executor


def tf_listener() -> typing.Optional[TFListenerWrapper]:
    """Gets the tf listener of the current ROS 2 aware scope, if any."""
    process = current()
    if process is None:
        return None
    return process.tf_listener


@typing.overload
def load(factory: NodeFactoryCallable[NodeT], *args: typing.Any, **kwargs: typing.Any) -> NodeT:
    """Loads a ROS 2 node within the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.load` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """


@typing.overload
def load(factory: GraphFactoryCallable[NodeT], *args: typing.Any, **kwargs: typing.Any) -> typing.List[NodeT]:
    """Loads a ROS 2 graph within the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.load` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """


def load(
    factory: typing.Union[
        NodeFactoryCallable[NodeT],
        GraphFactoryCallable[NodeT],
    ],
    *args: typing.Any,
    **kwargs: typing.Any,
) -> typing.Union[NodeT, typing.List[NodeT]]:
    """Loads a ROS 2 node (or a collection thereof) within the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.load` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    return process.load(factory, *args, **kwargs)


def unload(loaded: typing.Union[rclpy.node.Node, typing.List[rclpy.node.Node]]) -> None:
    """Unloads a ROS 2 node (or a collection thereof) from the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.unload` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    process.unload(loaded)


@typing.overload
def managed(
    factory: NodeFactoryCallable[NodeT],
    *args: typing.Any,
    **kwargs: typing.Any,
) -> typing.ContextManager[NodeT]:
    """Manages a ROS 2 node within the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.managed` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """


@typing.overload
def managed(
    factory: GraphFactoryCallable[NodeT],
    *args: typing.Any,
    **kwargs: typing.Any,
) -> typing.ContextManager[typing.List[NodeT]]:
    """Manages a ROS 2 graph within the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.managed` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """


def managed(
    factory: typing.Union[
        NodeFactoryCallable[NodeT],
        GraphFactoryCallable[NodeT],
    ],
    *args: typing.Any,
    **kwargs: typing.Any,
) -> typing.Union[typing.ContextManager[NodeT], typing.ContextManager[typing.List[NodeT]]]:
    """Manages a ROS 2 node (or a collection thereof) within the current ROS 2 aware process scope.

    See `ROSAwareProcess` and `ROSAwareScope.managed` documentation for further
    reference on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    return process.managed(factory, *args, **kwargs)


def spin(
    factory: typing.Optional[typing.Union[NodeFactoryCallable, GraphFactoryCallable]] = None,
    *args: typing.Any,
    **kwargs: typing.Any,
) -> None:
    """Spins current ROS 2 aware process executor (and all ROS 2 nodes in it).

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
    cli: typing.Optional[argparse.ArgumentParser] = None,
    **kwargs: typing.Any,
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
    """Wait for current ROS 2 aware process to shutdown.

    See `ROSAwareProcess.wait_for_shutdown` documentation for further reference
    on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    return process.wait_for_shutdown(timeout_sec=timeout_sec)


def wait_for_interrupt(*, timeout_sec: typing.Optional[float] = None) -> None:
    """Wait for current ROS 2 aware process interruption.

    See `ROSAwareProcess.wait_for_interrupt` documentation for further reference
    on positional and keyword arguments taken by this function.

    Raises:
        RuntimeError: if no process is executing.
    """
    process = current()
    if process is None:
        raise RuntimeError("no process is executing")
    return process.wait_for_interrupt(timeout_sec=timeout_sec)
