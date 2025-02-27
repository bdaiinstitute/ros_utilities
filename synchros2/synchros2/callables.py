# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

import abc
import functools
import inspect
from typing import (
    Any,
    Callable,
    Generic,
    Iterable,
    List,
    Literal,
    Optional,
    Tuple,
    Type,
    TypeVar,
    Union,
    cast,
    overload,
)

from rclpy.task import Future

from synchros2.executors import assign_coroutine
from synchros2.futures import AnyFuture, FutureLike, as_proper_future
from synchros2.utilities import fqn, take_kwargs


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

    def synchronous(self, *vargs: Iterable[Any], **kwargs: Any) -> List:
        """Invoke callable synchronously over a sequence of a argument tuples (zipped).

        A sequence of results is returned.
        """
        return [self.wrapped_callable.synchronous(*args, **kwargs) for args in zip(*vargs)]

    def asynchronous(self, *vargs: Iterable[Any], **kwargs: Any) -> Future:
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


class GeneralizedGuard(GeneralizedDecorator):
    """A decorator that guards generalized callable invocations."""

    def __init__(
        self,
        condition: Callable[[], bool],
        wrapped_callable: GeneralizedCallable,
        message: Optional[str] = None,
    ) -> None:
        """Initializes generalized guard.

        Args:
            condition: boolean predicate to guard invocations.
            wrapped_callable: the guarded callable.
            message: optional human-readable message to raise whenever
            the guard condition does not hold.
        """
        super().__init__(wrapped_callable)
        self.condition = condition
        if message is None:
            message = fqn(condition)
        self.message = message

    def synchronous(self, *args: Any, **kwargs: Any) -> Any:
        """Invokes callable synchronously if the guarded condition holds true, raises otherwise."""
        if not self.condition():
            raise RuntimeError(self.message)
        return self.wrapped_callable.synchronous(*args, **kwargs)

    def asynchronous(self, *args: Any, **kwargs: Any) -> Any:
        """Invokes callable asynchronously if the guarded condition holds true, raises otherwise."""
        if not self.condition():
            raise RuntimeError(self.message)
        return self.wrapped_callable.asynchronous(*args, **kwargs)


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
            if not method.transitional:
                if inspect.iscoroutinefunction(method.prototype):
                    self.asynchronous_callable = method.prototype
                else:
                    self.synchronous_callable = method.prototype
            if method.synchronous_overload is not None:
                self.synchronous_callable = method.synchronous_overload
            if method.asynchronous_overload is not None:
                self.asynchronous_callable = method.asynchronous_overload

            self.default_callable: Optional[Callable] = None
            if not method.transitional:
                if self.synchronous_callable is not None:
                    self.default_callable = self.synchronous_callable
                else:
                    self.default_callable = self.asynchronous_callable
            else:
                self.default_callable = method.legacy_overload

        def __get__(
            self,
            instance: Optional[Any],
            owner: Optional[Type] = None,
        ) -> Union["GeneralizedMethod.Unbound", "GeneralizedMethod.Bound"]:
            if instance is None:
                return self
            synchronous_callable = self.synchronous_callable
            if synchronous_callable is not None:
                synchronous_callable = synchronous_callable.__get__(instance, owner)
                assert synchronous_callable is not None
            asynchronous_callable = self.asynchronous_callable
            if asynchronous_callable is not None:
                asynchronous_callable = asynchronous_callable.__get__(instance, owner)
                assert asynchronous_callable is not None
                if inspect.iscoroutinefunction(self.asynchronous_callable):
                    asynchronous_callable = assign_coroutine(asynchronous_callable, instance.executor)
            default_callable = self.default_callable
            if default_callable is not None:
                default_callable = default_callable.__get__(instance, owner)
                assert default_callable is not None
                if inspect.iscoroutinefunction(self.default_callable):
                    default_callable = assign_coroutine(default_callable, instance.executor)
            implementation = GeneralizedFunction(synchronous_callable, asynchronous_callable)
            return GeneralizedMethod.Bound(implementation, default_callable)

    class Bound(VectorizingCallable, ComposableCallable):
        """A bound generalized method callable."""

        def __init__(self, body: GeneralizedCallable, default_callable: Optional[Callable] = None) -> None:
            """Initialize bound method callable.

            Args:
                body: method body as a generalized callable
                default_callable: optionally override default plain calls, defaults to synchronous calls.
            """
            self.body = body
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
        self.legacy_overload: Optional[Callable] = None
        if transitional:
            self.legacy_overload = prototype
        self.synchronous_overload: Optional[Callable] = None
        self.asynchronous_overload: Optional[Callable] = None

    @property
    def transitional(self) -> bool:
        """Check whether this method is transitional or not."""
        return self.legacy_overload is not None

    def legacy(self, func: Callable) -> Callable:
        """Register `func` as this method legacy overload."""
        if self.legacy_overload is not None:
            raise RuntimeError("cannot redefine legacy overload")
        self.legacy_overload = func
        return func

    def synchronous(self, func: Callable) -> Callable:
        """Register `func` as this method synchronous overload."""
        if self.synchronous_overload is not None:
            raise RuntimeError("cannot redefine synchronous overload")
        self.synchronous_overload = func
        return func

    sync_overload = synchronous

    def asynchronous(self, func: Callable) -> Callable:
        """Register `func` as this method asynchronous overload."""
        if self.asynchronous_overload is not None:
            raise RuntimeError("cannot redefine asynchronous overload")
        self.asynchronous_overload = func
        return func

    async_overload = asynchronous

    def __set_name__(self, owner: Type, name: str) -> None:
        self.__attribute_name = f"__{name}_method"
        setattr(owner, self.__attribute_name, GeneralizedMethod.Unbound(self))

    def rebind(self, instance: Any, body: GeneralizedCallable) -> None:
        """Change this method's `body` for the given `instance`."""
        default_callable: Optional[Callable] = None
        if self.legacy_overload is not None:
            default_callable = self.legacy_overload.__get__(instance)
        bound_method = GeneralizedMethod.Bound(body, default_callable)
        setattr(instance, self.__attribute_name, bound_method)

    def __get__(self, instance: Optional[Any], owner: Optional[Type] = None) -> Any:
        if instance is None:
            return self
        return getattr(instance, self.__attribute_name)


P = TypeVar("P")


class GeneralizedMethodLike(Generic[P], GeneralizedMethod):
    """A generalized method that can be type annotated via user-defined protocols."""

    @overload
    def __get__(
        self,
        instance: Literal[None],
        owner: Optional[Type] = ...,
    ) -> "GeneralizedMethodLike[P]":
        ...

    @overload
    def __get__(self, instance: Any, owner: Optional[Type] = ...) -> "P":
        ...

    def __get__(
        self,
        instance: Optional[Any],
        owner: Optional[Type] = None,
    ) -> Union["GeneralizedMethodLike[P]", "P"]:
        if instance is None:
            return self
        return cast(P, super().__get__(instance, owner))


@overload
def generalized_method(
    func: Callable,
    *,
    spec: Type[GeneralizedMethodLike[P]],
) -> GeneralizedMethodLike[P]:
    ...


@overload
def generalized_method(
    func: Literal[None] = None,
    *,
    spec: Type[GeneralizedMethodLike[P]],
    transitional: bool = ...,
) -> Callable[[Callable], GeneralizedMethodLike[P]]:
    ...


@overload
def generalized_method(func: Callable, *, transitional: bool = ...) -> GeneralizedMethod:
    ...


@overload
def generalized_method(
    func: Literal[None] = None,
    *,
    transitional: bool = ...,
) -> Callable[[Callable], GeneralizedMethod]:
    ...


def generalized_method(
    func: Optional[Callable] = None,
    *,
    spec: Optional[Type[GeneralizedMethodLike[P]]] = None,
    transitional: bool = False,
) -> Union[
    GeneralizedMethod,
    GeneralizedMethodLike[P],
    Callable[[Callable], GeneralizedMethod],
    Callable[[Callable], GeneralizedMethodLike[P]],
]:
    """Define a generalized method by decoration.

    Args:
        func: method function, usually just a signature but
        may also be used as an overload for convenience.
        spec: optional type annotated specification.
        transitional: a transitional method will stick to its
        prototype for default invocations.
    """
    if spec is not None:

        def __decorator_with_spec(func: Callable) -> GeneralizedMethodLike[P]:
            assert spec is not None
            return spec(func, transitional)

        if func is None:
            return __decorator_with_spec
        return __decorator_with_spec(func)

    def __decorator(func: Callable) -> GeneralizedMethod:
        return GeneralizedMethod(func, transitional)

    if func is None:
        return __decorator
    return __decorator(func)
