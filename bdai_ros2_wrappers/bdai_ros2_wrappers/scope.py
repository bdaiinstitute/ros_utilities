# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import contextlib
import os
import threading
import typing

import rclpy
import rclpy.executors
import rclpy.logging
import rclpy.node

from bdai_ros2_wrappers.executors import AutoScalingMultiThreadedExecutor, background, foreground
from bdai_ros2_wrappers.logging import logs_to_ros
from bdai_ros2_wrappers.node import Node
from bdai_ros2_wrappers.utilities import fqn, namespace_with

AnyEntity = typing.Union[rclpy.node.Node, typing.List[rclpy.node.Node]]
NodeFactoryCallable = typing.Callable[..., rclpy.node.Node]
GraphFactoryCallable = typing.Callable[..., typing.Iterable[rclpy.node.Node]]
AnyEntityFactoryCallable = typing.Union[NodeFactoryCallable, GraphFactoryCallable]


class ROSAwareScope(typing.ContextManager["ROSAwareScope"]):
    """
    A context manager to enable ROS 2 code in a given scope by providing
    an `rclpy` node and an autoscaling multi-threaded executor spinning
    in a background thread.
    """

    global_: typing.Optional["ROSAwareScope"] = None

    class LocalStack(threading.local):
        top: typing.Optional["ROSAwareScope"] = None

    local = LocalStack()

    def __init__(
        self,
        *,
        prebaked: bool = True,
        global_: bool = False,
        namespace: typing.Optional[str] = None,
        autospin: typing.Optional[bool] = None,
        forward_logging: bool = False,
        context: typing.Optional[rclpy.context.Context] = None,
    ) -> None:
        """
        Initializes the ROS 2 aware scope.

        Args:
            prebaked: whether to include an implicit main node in the scope graph
            or not, for convenience. Prebaked scopes must autospin.
            global_: whether to make this scope global (ie. accessible from all threads).
            Only one, outermost global scope can be entered at any given time. Global
            scopes can only be entered from the main thread.
            namespace: optional namespace for all underlying nodes. Defaults to
            a unique hidden namespace.
            autospin: whether to automatically instantiate and push an executor
            to a background thread on scope entry or not.
            forward_logging: whether to forward `logging` logs to the ROS 2 logging
            system or not. Defaults to False.
            context: optional context for the underlying executor and nodes.
        """
        if autospin is None:
            autospin = prebaked
        if prebaked and not autospin:
            raise ValueError("prebaked scope must autospin")
        if namespace is None:
            namespace = f"_ros_aware_scope_{os.getpid()}_{id(self)}"
        self._namespace = namespace
        self._context = context
        self._prebaked = prebaked
        self._autospin = autospin
        self._forward_logging = forward_logging
        self._global = global_
        self._lock = threading.Lock()
        self._node: typing.Optional[rclpy.node.Node] = None
        self._graph: typing.List[rclpy.node.Node] = []
        self._executor: typing.Optional[rclpy.executors.Executor] = None
        self._stack: typing.Optional[contextlib.ExitStack] = None
        self._owner: typing.Optional[threading.Thread] = None
        self._parent: typing.Optional[ROSAwareScope] = None

    @property
    def context(self) -> typing.Optional[rclpy.context.Context]:
        """Gets scope context (may be ``None`` if default)."""
        return self._context

    def __enter__(self) -> "ROSAwareScope":
        """
        Enters scope.

        Scopes are not reentrant.

        Raises:
            RuntimeError: if scope has been entered already, or, if global, when
            attempting to enter from a thread other than the main thread, when
            a global scope has already been entered, and when scope is not the
            outermost scope.
        """
        with self._lock:
            if self._stack is not None:
                raise RuntimeError("scope has been entered already")

            if self._global:
                if threading.current_thread() is not threading.main_thread():
                    raise RuntimeError("a global scope can only be entered from main thread")
                if ROSAwareScope.global_ is not None:
                    raise RuntimeError("a global scope has been entered already")
                if ROSAwareScope.local.top is not None:
                    raise RuntimeError("a global scope must be entered before any local scopes")

            self._stack = contextlib.ExitStack()
            self._owner = threading.current_thread()

            if self._autospin:
                logger = rclpy.logging.get_logger(self._namespace.replace("/", ".").strip("."))
                self._executor = self._stack.enter_context(
                    background(AutoScalingMultiThreadedExecutor(logger=logger, context=self._context))
                )

                if self._prebaked:
                    node = Node("node", namespace=self._namespace, context=self._context)
                    self._executor.add_node(node)
                    self._graph.append(node)
                    if self._forward_logging:
                        self._stack.enter_context(logs_to_ros(node))
                    self._node = node

            if self._global:
                ROSAwareScope.global_ = self
            self._parent = ROSAwareScope.local.top
            ROSAwareScope.local.top = self
            return self

    def __exit__(self, *exc: typing.Any) -> None:
        """
        Exits scope.

        Raises:
            RuntimeError: if scope has not been entered (or already exited)
            or if scope was entered from a different thread.
        """
        with self._lock:
            if self._stack is None:
                raise RuntimeError("scope has not been entered (or already exited)")
            if self._owner is not threading.current_thread():
                raise RuntimeError("scope entered from a different thread")
            ROSAwareScope.local.top = self._parent
            if self._global:
                ROSAwareScope.global_ = None
            self._parent = None
            self._stack.close()
            self._stack = None
            self._owner = None
            assert self._executor is not None
            for node in self._graph:
                self._executor.remove_node(node)
                node.destroy_node()
            self._graph.clear()
            self._executor = None
            self._node = None

    @property
    def graph(self) -> typing.List[rclpy.node.Node]:
        """Gets scope nodes, if any."""
        return list(self._graph)

    @property
    def executor(self) -> typing.Optional[rclpy.executors.Executor]:
        """Gets scope executor, if any."""
        return self._executor

    @executor.setter
    def executor(self, executor: rclpy.executors.Executor) -> None:
        """
        Sets scope executor.

        Args:
            executor: executor to be used.

        Raises:
            RuntimeError: if scope has not been entered (or already exited)
            or an scope executor is already set.
        """
        with self._lock:
            if self._stack is None:
                raise RuntimeError("scope has not been entered (or already exited)")
            if self._executor is not None:
                raise RuntimeError("scope executor already set")
            self._executor = executor

    @property
    def node(self) -> typing.Optional[rclpy.node.Node]:
        """Gets scope main node, if any."""
        return self._node

    @node.setter
    def node(self, node: rclpy.node.Node) -> None:
        """
        Sets scope main node.

        Args:
            node: node to be promoted to main node. It must be a node known to the scope.

        Raises:
            RuntimeError: if the scope has not been entered (or already exited), if a
            a scope main node has been set already, or if the node is foreign to the scope.
        """
        with self._lock:
            if self._stack is None:
                raise RuntimeError("scope has not been entered (or already exited)")
            if self._node is not None:
                raise RuntimeError("main scope node already set")
            if node not in self._graph:
                raise RuntimeError("node is not in scope")
            if self._forward_logging:
                self._stack.enter_context(logs_to_ros(node))
            self._node = node

    @typing.overload
    def managed(
        self, factory: NodeFactoryCallable, *args: typing.Any, **kwargs: typing.Any
    ) -> typing.ContextManager[rclpy.node.Node]:
        """
        Manages a ROS 2 node within scope.

        Upon context entry, a ROS 2 node is instantiated and loaded.
        Upon context exit, that ROS 2 node is unloaded and destroyed.

        See `ROSAwareScope.load` documentation for further reference
        on positional and keyword arguments taken by this method.

        Returns:
            a context manager for the loaded node.
        """

    @typing.overload
    def managed(
        self, factory: GraphFactoryCallable, *args: typing.Any, **kwargs: typing.Any
    ) -> typing.ContextManager[typing.List[rclpy.node.Node]]:
        """
        Manages a collection (or graph) of ROS 2 nodes within scope.

        Upon context entry, ROS 2 nodes are instantiated and loaded.
        Upon context exit, those ROS 2 nodes are unloaded and destroyed.

        See `ROSAwareScope.load` documentation for further reference
        on positional and keyword arguments taken by this method.

        Returns:
            a context manager for the loaded nodes.
        """

    @contextlib.contextmanager
    def managed(
        self, factory: AnyEntityFactoryCallable, *args: typing.Any, **kwargs: typing.Any
    ) -> typing.Iterator[AnyEntity]:
        loaded = self.load(factory, *args, **kwargs)
        try:
            yield loaded
        finally:
            self.unload(loaded)

    @typing.overload
    def load(self, factory: NodeFactoryCallable, *args: typing.Any, **kwargs: typing.Any) -> rclpy.node.Node:
        """
        Instantiates and loads a ROS 2 node.

        Args:
            factory: callable to instantiate a ROS 2 node.
            It is expected to accept `rclpy.node.Node` arguments.
            Note it can be an `rclpy.node.Node` subclass object.
            args: optional positional arguments for ``node_factory``.
            kwargs: optional keyword arguments for ``node_factory``.

        Returns:
            loaded ROS 2 node.

        Raises:
            RuntimeError: if scope has not been entered (or already exited)
            or an scope executor has not been set yet.
        """

    @typing.overload
    def load(
        self, factory: GraphFactoryCallable, *args: typing.Any, **kwargs: typing.Any
    ) -> typing.List[rclpy.node.Node]:
        """
        Instantiates and loads a collection (or graph) of ROS 2 nodes.

        Args:
            factory: callable to instantiate a collection of ROS 2 nodes.
            It is expected to accept `rclpy.node.Node` arguments.
            args: optional positional arguments for `graph_factory`.
            kwargs: optional keyword arguments for `graph_factory`.

        Returns:
            loaded ROS 2 nodes.

        Raises:
            RuntimeError: if scope has not been entered (or already exited) or
            an scope executor has not been set yet.
        """

    def load(
        self,
        factory: AnyEntityFactoryCallable,
        *args: typing.Any,
        namespace: typing.Optional[str] = None,
        **kwargs: typing.Any,
    ) -> AnyEntity:
        with self._lock:
            if self._stack is None:
                raise RuntimeError("scope has not been entered (or was already exited)")
            if self._executor is None:
                raise RuntimeError("scope executor has not been set")
            if namespace is not None:
                namespace = namespace_with(self._namespace, namespace)
            else:
                namespace = self._namespace
            node_or_graph = factory(*args, context=self._context, namespace=namespace, **kwargs)
            if not isinstance(node_or_graph, rclpy.node.Node):
                graph = list(node_or_graph)
                for node in graph:
                    self._executor.add_node(node)
                self._graph.extend(graph)
                return graph
            node = node_or_graph
            self._executor.add_node(node)
            self._graph.append(node)
            return node

    def unload(self, loaded: AnyEntity) -> None:
        """
        Unloads and destroys ROS 2 nodes.

        Args:
            loaded: ROS 2 node or a collection thereof to unload.

        Raises:
            RuntimeError: if scope has not been entered (or already exited) or
            an scope executor has not been set yet.
            ValueError: if any given ROS 2 node is not found in the scope graph.
        """
        with self._lock:
            if self._stack is None:
                raise RuntimeError("scope has not been entered (or already exited)")
            if self._executor is None:
                raise RuntimeError("scope executor has not been set")
            graph = [loaded] if isinstance(loaded, rclpy.node.Node) else loaded
            if any(node for node in graph if node not in self._graph):
                raise ValueError("node unknown (unloaded already?)")
            for node in graph:
                self._executor.remove_node(node)
                self._graph.remove(node)
                node.destroy_node()

    @typing.overload
    def spin(self) -> None:
        """
        Spins scope executor (and all nodes in it).

        If no scope executor was set, a default one is.

        Raises:
            RuntimeError: if scope has not been entered (or already exited) or
            the scope executor was configured to spin automatically.
        """

    @typing.overload
    def spin(self, factory: NodeFactoryCallable, *args: typing.Any, **kwargs: typing.Any) -> None:
        """
        Spins scope executor (and all nodes in it).

        Additionally, and for as long as it spins, a ROS 2 node is loaded.
        If no scope executor was set, a default one is.

        Args:
            factory: callable to instantiate a ROS 2 node.
            It is expected to accept `rclpy.node.Node` arguments.
            Note it can be an `rclpy.node.Node` subclass object.
            args: optional positional arguments for ``node_factory``.
            kwargs: optional keyword arguments for ``node_factory``.

        Raises:
            RuntimeError: if scope has not been entered (or already exited) or
            the scope executor was configured to spin automatically.
        """

    @typing.overload
    def spin(self, factory: GraphFactoryCallable, *args: typing.Any, **kwargs: typing.Any) -> None:
        """
        Spins scope executor (and all nodes in it).

        Additionally, and for as long as it spins, a collection of ROS 2 nodes is loaded.
        If no scope executor was set, a default one is.

        Args:
            factory: callable to instantiate a collection of ROS 2 nodes.
            It is expected to accept `rclpy.node.Node` arguments.
            args: optional positional arguments for `graph_factory`.
            kwargs: optional keyword arguments for `graph_factory`.

        Raises:
            RuntimeError: if scope has not been entered (or already exited) or
            the scope executor was configured to spin automatically.
        """

    def spin(
        self, factory: typing.Optional[AnyEntityFactoryCallable] = None, *args: typing.Any, **kwargs: typing.Any
    ) -> None:
        if self._stack is None:
            raise RuntimeError("scope has not been entered (or already exited)")
        if self._autospin:
            raise RuntimeError("scope executor already spinning")
        with self._lock:
            if self._executor is None:
                logger = rclpy.logging.get_logger(self._namespace or fqn(self.__class__))
                self._executor = self._stack.enter_context(
                    foreground(AutoScalingMultiThreadedExecutor(logger=logger, context=self._context))
                )
        with contextlib.ExitStack() as stack:
            if factory is not None:
                cm = self.managed(factory, *args, **kwargs)
                stack.enter_context(cm)
            self._executor.spin()


@contextlib.contextmanager
def top(
    args: typing.Optional[typing.Sequence[str]] = None,
    *,
    context: typing.Optional[rclpy.context.Context] = None,
    global_: bool = False,
    **kwargs: typing.Any,
) -> typing.Iterator[ROSAwareScope]:
    """
    Manages a ROS 2 aware scope, handling ROS 2 context lifecycle as well.

    Args:
        args: optional command-line arguments for context initialization.
        context: optional context to manage. If none is provided, one will
        be created. For global scopes, the default context will be used.

    See `ROSAwareScope` documentation for further reference on positional
    and keyword arguments taken by this function.

    Returns:
        a context manager.
    """
    if context is None and not global_:
        context = rclpy.context.Context()
    rclpy.init(args=args, context=context)
    try:
        with ROSAwareScope(global_=global_, context=context, **kwargs) as scope:
            yield scope
    finally:
        rclpy.try_shutdown(context=context)


def current() -> typing.Optional[ROSAwareScope]:
    """Gets the current ROS 2 aware scope, if any."""
    return getattr(ROSAwareScope.local, "top", ROSAwareScope.global_)


def node() -> typing.Optional[rclpy.node.Node]:
    """Gets the node of the current ROS 2 aware scope, if any."""
    scope = current()
    if scope is None:
        return None
    return scope.node


def executor() -> typing.Optional[rclpy.executors.Executor]:
    """Gets the executor of the current ROS 2 aware scope, if any."""
    scope = current()
    if scope is None:
        return None
    return scope.executor


def load(factory: AnyEntityFactoryCallable, *args: typing.Any, **kwargs: typing.Any) -> AnyEntity:
    """
    Loads a ROS 2 node (or a collection thereof) within the current ROS 2 aware scope.

    See `ROSAwareScope.load` documentation for further reference on positional and keyword
    arguments taken by this function.

    Raises:
        RuntimeError: if called outside scope.
    """
    scope = current()
    if scope is None:
        raise RuntimeError("not in any scope")
    return scope.load(factory, *args, **kwargs)


def unload(loaded: AnyEntity) -> None:
    """
    Unloads a ROS 2 node (or a collection thereof) from the current ROS 2 aware scope.

    See `ROSAwareScope.unload` documentation for further reference on positional and
    keyword arguments taken by this function.

    Raises:
        RuntimeError: if called outside scope.
    """
    scope = current()
    if scope is None:
        raise RuntimeError("not in any scope")
    scope.unload(loaded)


def managed(
    factory: AnyEntityFactoryCallable, *args: typing.Any, **kwargs: typing.Any
) -> typing.ContextManager[AnyEntity]:
    """
    Manages a ROS 2 node (or a collection thereof) within the current ROS 2 aware scope.

    See `ROSAwareScope.managed` documentation for further reference on positional and
    keyword arguments taken by this function.

    Raises:
        RuntimeError: if called outside scope.
    """
    scope = current()
    if scope is None:
        raise RuntimeError("not in any scope")
    return scope.managed(factory, *args, **kwargs)


def spin(factory: typing.Optional[AnyEntityFactoryCallable] = None, *args: typing.Any, **kwargs: typing.Any) -> None:
    """
    Spins current ROS 2 aware scope executor (and all the ROS 2 nodes in it).

    Optionally, manages a ROS 2 node (or a collection thereof) for as long as it spins.

    See `ROSAwareScope.spin` documentation for further reference on positional and keyword
    arguments taken by this function.

    Raises:
        RuntimeError: if called outside scope.
    """
    scope = current()
    if scope is None:
        raise RuntimeError("not in any scope")
    if factory is not None:
        scope.spin(factory, *args, **kwargs)
    else:
        scope.spin()
