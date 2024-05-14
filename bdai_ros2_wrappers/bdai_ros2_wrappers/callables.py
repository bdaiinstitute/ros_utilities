# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

import abc
import functools
from typing import Any, Callable, Iterable, Optional, Tuple

from rclpy.task import Future

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
