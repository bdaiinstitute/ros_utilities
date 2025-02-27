# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

from collections.abc import Mapping
from typing import Any, Optional

import pytest

from synchros2.callables import GeneralizedGuard, generalized_method
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope


class Bucket:
    def __init__(
        self,
        ros: ROSAwareScope,
        storage: Optional[Mapping] = None,
    ) -> None:
        self.executor = ros.executor
        if storage is None:
            storage = {}
        self._storage = dict(storage)

    @generalized_method(transitional=True)
    def create(self, content: Any) -> str:
        name = str(hash(content))
        if name in self._storage:
            raise RuntimeError()
        self._storage[name] = content
        return name

    @create.sync_overload
    def _create_sync(self, name: str, content: Any) -> bool:
        if name in self._storage:
            return False
        self._storage[name] = content
        return True

    @create.async_overload
    async def _create_async(self, name: str, content: Any) -> bool:
        return self._create_sync(name, content)

    @generalized_method
    def read(self, name: str) -> Optional[Any]:
        return self._storage.get(name)

    @read.async_overload
    async def _read_async(self, name: str) -> Optional[Any]:
        return self.read(name)

    @generalized_method
    async def update(self, name: str, content: Any) -> bool:
        if name not in self._storage:
            return False
        self._storage[name] = content
        return True

    @generalized_method
    def delete(self, name: str) -> bool:
        if name not in self._storage:
            return False
        del self._storage[name]
        return True


def test_transitional_method(ros: ROSAwareScope) -> None:
    bucket = Bucket(ros)
    name = bucket.create("some data")
    assert name in bucket._storage
    assert bucket._storage[name] == "some data"

    with pytest.raises(RuntimeError):
        bucket.create("some data")

    assert not bucket.create.synchronously(name, "some other data")
    assert bucket.create.synchronously("my-data", "some other data")
    assert "my-data" in bucket._storage
    assert bucket._storage["my-data"] == "some other data"

    future = bucket.create.asynchronously("my-data", "more data")
    assert wait_for_future(future, timeout_sec=5.0)
    assert future.result() is False

    future = bucket.create.asynchronously("extras", "more data")
    assert wait_for_future(future, timeout_sec=5.0)
    assert future.result() is True
    assert "extras" in bucket._storage
    assert bucket._storage["extras"] == "more data"


def test_nominal_method(ros: ROSAwareScope) -> None:
    bucket = Bucket(ros, {"my-data": "some data"})

    assert bucket.read("my-data") == "some data"
    assert bucket.read.synchronously("my-data") == "some data"
    future = bucket.read.asynchronously("my-data")
    assert wait_for_future(future, timeout_sec=5.0)
    assert future.result() == "some data"

    assert not bucket.read("other-data")
    assert not bucket.read.synchronously("other-data")
    future = bucket.read.asynchronously("other-data")
    assert wait_for_future(future, timeout_sec=5.0)
    assert future.result() is None


def test_sync_only_method(ros: ROSAwareScope) -> None:
    bucket = Bucket(
        ros,
        {
            "my-data": "some data",
            "extras": "more data",
            "old": "old data",
        },
    )
    assert bucket.delete("my-data")
    assert "my-data" not in bucket._storage
    assert not bucket.delete("my-data")
    assert bucket.delete.synchronously("extras")
    assert "extras" not in bucket._storage
    assert not bucket.delete.synchronously("extras")
    with pytest.raises(NotImplementedError):
        bucket.delete.asynchronously("old")
    assert "old" in bucket._storage


def test_async_only_method(ros: ROSAwareScope) -> None:
    bucket = Bucket(ros, {"my-data": "some data"})
    future = bucket.update("my-data", "new data")
    assert wait_for_future(future, timeout_sec=5.0)
    assert future.result() is True
    assert bucket._storage["my-data"] == "new data"

    future = bucket.update.asynchronously("my-data", "newer data")
    assert wait_for_future(future, timeout_sec=5.0)
    assert future.result() is True
    assert bucket._storage["my-data"] == "newer data"

    with pytest.raises(NotImplementedError):
        bucket.update.synchronously("my-data", "")
    assert bucket._storage["my-data"] == "newer data"


def test_vectorized_method(ros: ROSAwareScope) -> None:
    bucket = Bucket(
        ros,
        {
            "my-data": "some data",
            "extras": "more data",
        },
    )
    data = bucket.read.vectorized(["my-data", "extras"])
    assert data == ["some data", "more data"]

    data = bucket.read.vectorized.synchronously(["my-data", "extras"])
    assert data == ["some data", "more data"]

    future = bucket.read.vectorized.asynchronously(["my-data", "extras"])
    assert wait_for_future(future, timeout_sec=5.0)
    assert future.result() == ["some data", "more data"]


def test_composed_method(ros: ROSAwareScope) -> None:
    bucket = Bucket(ros)
    Bucket.create.rebind(
        bucket,
        bucket.create.compose(
            (lambda name, *data: (name, data)),
            starred=True,
        ),
    )
    name = bucket.create("some data")
    assert name in bucket._storage
    assert bucket._storage[name] == "some data"

    assert bucket.create.synchronously("my-data", "some other data", 1, True)
    assert "my-data" in bucket._storage
    assert bucket._storage["my-data"] == ("some other data", 1, True)

    future = bucket.create.asynchronously("extras", 0, "more data", False)
    assert wait_for_future(future, timeout_sec=5.0)
    assert future.result() is True
    assert "extras" in bucket._storage
    assert bucket._storage["extras"] == (0, "more data", False)


def test_guarded_method(ros: ROSAwareScope) -> None:
    bucket = Bucket(ros, {"my-data": "some data"})

    read_permission = False

    def allowed() -> bool:
        nonlocal read_permission
        return read_permission

    guarded_read = GeneralizedGuard(allowed, bucket.read)
    Bucket.read.rebind(bucket, guarded_read)

    with pytest.raises(RuntimeError):
        bucket.read("my-data")

    read_permission = True

    assert bucket.read("my-data") == "some data"
