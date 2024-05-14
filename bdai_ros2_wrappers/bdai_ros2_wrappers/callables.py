# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

import abc
import functools
import inspect
from typing import Any, Callable, Iterable, Optional, Tuple, Type, Union, overload

from rclpy.task import Future

from bdai_ros2_wrappers.executors import assign_coroutine
from bdai_ros2_wrappers.futures import AnyFuture, FutureLike, as_proper_future
from bdai_ros2_wrappers.utilities import take_kwargs


def starmap_async(func: Callable[..., AnyFuture], iterable: Iterable[Tuple[Any, ...]]) -> Future:
    """Transform an iterable of tuples by star expanding them and invoking an asynchronous `func`.

    Args:
        func: a callable with a future-like return value.
        iterable: an iterable of tuples of positional arguments for `func`.

    Returns:
        a future-like object to the transformed iterable.
    """
    results = []
    args = iter(iterable)
    aggregate_future = Future()

    def consume_next(future: Optional[FutureLike] = None) -> None:
        nonlocal args, results, aggregate_future
        if future is not None:
            if future.exception():
                aggregate_future.set_exception(future.exception())
                return
            results.append(future.result())
        try:
            proper_future = as_proper_future(func(*next(args)))
            proper_future.add_done_callback(consume_next)
        except StopIteration:
            aggregate_future.set_result(results)

    consume_next()

    return aggregate_future


class GeneralizedCallable(abc.ABC):
    """A generalized callable that allows synchronous and asynchronous execution."""

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        """Forward call to `GeneralizedCallable.synchronous`."""
        return self.synchronous(*args, **kwargs)

    @abc.abstractmethod
    def synchronous(self, *args: Any, **kwargs: Any) -> Any:
        """Invoke callable synchronously (ie. potentially blocking)."""
        ...

    def synchronously(self, *args: Any, **kwargs: Any) -> Any:
        """Alias of `GeneralizedCallable.synchronous`."""
        return self.synchronous(*args, **kwargs)

    @abc.abstractmethod
    def asynchronous(self, *args: Any, **kwargs: Any) -> Any:
        """Invoke callable asynchronously, returning a future-like object."""
        ...

    def asynchronously(self, *args: Any, **kwargs: Any) -> Any:
        """Alias of `GeneralizedCallable.asynchronous`."""
        return self.asynchronous(*args, **kwargs)


class GeneralizedDecorator(GeneralizedCallable):
    """A decorator for generalized callables."""

    def __init__(self, wrapped_callable: GeneralizedCallable) -> None:
        """Initializes the decorator, wrapping another callable."""
        self.wrapped_callable = wrapped_callable

    def __getattr__(self, name: str) -> Any:
        return getattr(self.wrapped_callable, name)


class VectorizingCallable(GeneralizedCallable):
    """A generalized callable that can be vectorized."""

    @property
    def vectorized(self) -> "VectorizedCallable":
        """Get a vectorized version of this callable."""
        return VectorizedCallable(self)


class ComposableCallable(GeneralizedCallable):
    """A generalized callable that can be composed."""

    def compose(self, func: Callable, starred: bool = False) -> "ComposedCallable":
        """Compose this callable with the given `func`.

        Args:
            func: callable to be composed, assumed synchronous.
            starred: whether the `func` call return value requires
            a star expansion for composition or not.

        Returns:
            the composed generalized callable.
        """
        return ComposedCallable(self, func, starred)


class VectorizedCallable(GeneralizedDecorator, ComposableCallable, VectorizingCallable):
    """A vectorization decorator that aggregates multiple invocations sequentially."""

    def synchronous(self, *vargs: Iterable[Any], **kwargs: Any) -> Any:
        """Invoke callable synchronously over a sequence of a argument tuples (zipped).

        A sequence of results is returned.
        """
        return [self.wrapped_callable.synchronous(*args, **kwargs) for args in zip(*vargs)]

    def asynchronous(self, *vargs: Iterable[Any], **kwargs: Any) -> Any:
        """Invoke callable asynchronously over a sequence of a argument tuples (zipped).

        A future to a sequence of results is returned.
        """
        return starmap_async(functools.partial(self.wrapped_callable.asynchronous, **kwargs), zip(*vargs))


class ComposedCallable(GeneralizedDecorator, ComposableCallable, VectorizingCallable):
    """A composition decorator that combines a generalized callable and a regular callable."""

    def __init__(
        self,
        wrapped_callable: GeneralizedCallable,
        composed_callable: Callable,
        starred: bool = False,
    ) -> None:
        """Initializes composed callable.

        Args:
            wrapped_callable: the left hand side (or outer) callable.
            composed_callable: the right hand side (or inner) callable.
            starred: whether the inner call return value requires a star
            expansion for composition or not.
        """
        super().__init__(wrapped_callable)
        self.composed_callable = composed_callable
        self.starred = starred

    def synchronous(self, *args: Any, **kwargs: Any) -> Any:
        """Invoke callable synchronously with the result of invoking the composed callable.

        Keyword arguments that match the composed callable signature will be forwarded to it.
        """
        inner_kwargs, outer_kwargs = take_kwargs(self.composed_callable, kwargs)
        if self.starred:
            return self.wrapped_callable.synchronous(*self.composed_callable(*args, **inner_kwargs), **outer_kwargs)
        return self.wrapped_callable.synchronous(self.composed_callable(*args, **inner_kwargs), **outer_kwargs)

    def asynchronous(self, *args: Any, **kwargs: Any) -> Any:
        """Invoke callable asynchronously with the result of invoking the composed callable.

        Keyword arguments that match the composed callable signature will be forwarded to it.
        """
        inner_kwargs, outer_kwargs = take_kwargs(self.composed_callable, kwargs)
        if self.starred:
            return self.wrapped_callable.asynchronous(*self.composed_callable(*args, **inner_kwargs), **outer_kwargs)
        return self.wrapped_callable.asynchronous(self.composed_callable(*args, **inner_kwargs), **outer_kwargs)


class GeneralizedFunction(GeneralizedCallable):
    """A generalized callable defined by parts."""

    def __init__(
        self,
        synchronous_callable: Optional[Callable] = None,
        asynchronous_callable: Optional[Callable] = None,
    ) -> None:
        """Initialize generalized function.

        Args:
            synchronous_callable: optional synchronous body.
            asynchronous_callable: optional asynchronous body.
        """
        self._synchronous_callable = synchronous_callable
        self._asynchronous_callable = asynchronous_callable

    def synchronous(self, *args: Any, **kwargs: Any) -> Any:
        """Invoke function synchronously (ie. potentially blocking)."""
        if self._synchronous_callable is None:
            raise NotImplementedError("synchronous invocation is not supported")
        return self._synchronous_callable(*args, **kwargs)

    def asynchronous(self, *args: Any, **kwargs: Any) -> Any:
        """Invoke function asynchronously, returning a future-like object."""
        if self._asynchronous_callable is None:
            raise NotImplementedError("asynchronous invocation is not supported")
        return self._asynchronous_callable(*args, **kwargs)


class GeneralizedMethod:
    """A data descriptor for generalized callables bound to class instances."""

    class Unbound:
        """An unbound generalized method descriptor."""

        def __init__(self, method: "GeneralizedMethod") -> None:
            """Initialize unbound descriptor.

            Args:
                method: associated generalized method.
            """
            self.synchronous_callable: Optional[Callable] = None
            self.asynchronous_callable: Optional[Callable] = None
            self.transitional_callable: Optional[Callable] = None
            if not method.transitional:
                if inspect.iscoroutinefunction(method.prototype):
                    self.asynchronous_callable = method.prototype
                else:
                    self.synchronous_callable = method.prototype
            else:
                self.transitional_callable = method.prototype
            if method.synchronous_overload is not None:
                self.synchronous_callable = method.synchronous_overload
            if method.asynchronous_overload is not None:
                self.asynchronous_callable = method.asynchronous_overload

        def __get__(
            self,
            instance: Optional[Any],
            owner: Optional[Type] = None,
        ) -> Union["GeneralizedMethod.Unbound", "GeneralizedMethod.Bound"]:
            if instance is None:
                return self
            synchronous_callable = self.synchronous_callable
            if synchronous_callable is not None:
                asynchronous_callable = synchronous_callable.__get__(instance, owner)
            asynchronous_callable = self.asynchronous_callable
            if asynchronous_callable is not None:
                asynchronous_callable = asynchronous_callable.__get__(instance, owner)
                if inspect.iscoroutinefunction(self.asynchronous_callable):
                    asynchronous_callable = assign_coroutine(asynchronous_callable, instance.executor)
            implementation = GeneralizedFunction(synchronous_callable, asynchronous_callable)
            transitional_callable = self.transitional_callable
            if transitional_callable is not None:
                transitional_callable = transitional_callable.__get__(instance, owner)
            return GeneralizedMethod.Bound(implementation, migrating_from=transitional_callable)

    class Bound(VectorizingCallable, ComposableCallable):
        """A bound generalized method callable."""

        def __init__(self, body: GeneralizedCallable, migrating_from: Optional[Callable] = None) -> None:
            """Initialize bound method callable.

            Args:
                body: method body as a generalized callable
                migrating_from: when migrating to generalized methods,
                the prior definition may be fed here so as to keep plain
                method invocations the same.
            """
            self.body = body
            default_callable = migrating_from
            if default_callable is None:
                default_callable = body.synchronous
            self._default_callable = default_callable

        def __getattr__(self, name: str) -> Any:
            return getattr(self.body, name)

        def __call__(self, *args: Any, **kwargs: Any) -> Any:
            """Invoke method (optionally pre-existing)."""
            return self._default_callable(*args, **kwargs)

        def synchronous(self, *args: Any, **kwargs: Any) -> Any:
            """Invoke method synchronously."""
            return self.body.synchronous(*args, **kwargs)

        def asynchronous(self, *args: Any, **kwargs: Any) -> Any:
            """Invoke method asynchronously."""
            return self.body.asynchronous(*args, **kwargs)

    def __init__(self, prototype: Callable, transitional: bool) -> None:
        """Initializes the generalized method.

        Args:
            prototype: method prototype, usually just a signature but
            may also be used as an overload for convenience (iff the
            function type matches the missing overload).
            transitional: a transitional method will stick to its
            prototype for default invocations, simplifying the
            adoption of generalized methods in existing codebases.
        """
        self.prototype = prototype
        self.transitional = transitional
        self.synchronous_overload: Optional[Callable] = None
        self.asynchronous_overload: Optional[Callable] = None

    def sync_overload(self, func: Callable) -> None:
        """Register `func` as this method synchronous overload."""
        if self.synchronous_overload is not None:
            raise RuntimeError("cannot redefine synchronous overload")
        self.synchronous_overload = func

    def async_overload(self, func: Callable) -> None:
        """Register `func` as this method asynchronous overload."""
        if self.asynchronous_overload is not None:
            raise RuntimeError("cannot redefine asynchronous overload")
        self.asynchronous_overload = func

    def __set_name__(self, owner: Type, name: str) -> None:
        self.__attribute_name = f"__{name}_method"
        setattr(owner, self.__attribute_name, GeneralizedMethod.Unbound(self))

    def rebind(self, instance: Any, body: GeneralizedCallable) -> None:
        """Change this method's `body` for the given `instance`."""
        bound_method = GeneralizedMethod.Bound(
            body,
            migrating_from=(self.prototype.__get__(instance) if self.transitional else None),
        )
        setattr(instance, self.__attribute_name, bound_method)

    def __get__(
        self,
        instance: Optional[Any],
        owner: Optional[Type] = None,
    ) -> Union["GeneralizedMethod", "GeneralizedMethod.Bound"]:
        if instance is None:
            return self
        return getattr(instance, self.__attribute_name)


@overload
def generalizedmethod(func: Callable, *, transitional: bool = ...) -> GeneralizedMethod:
    ...


@overload
def generalizedmethod(*, transitional: bool = ...) -> Callable:
    ...


def generalizedmethod(
    func: Optional[Callable] = None,
    *,
    transitional: bool = False,
) -> Union[Callable, GeneralizedMethod]:
    """Define a generalized method by decoration."""

    def _decorator(func: Callable) -> GeneralizedMethod:
        return GeneralizedMethod(func, transitional)

    if func is None:
        return _decorator
    return _decorator(func)
