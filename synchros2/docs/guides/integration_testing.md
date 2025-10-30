
# Integration Testing with synchros2

Unit testing ROS 2 code is similar to other software, but some care must be exercised due to non-determinism and peer discovery. Use `synchros2.scope.top` for test fixtures and unique domain IDs for isolation. Synchronization primitives and timeouts are recommended for reliable tests.

## Considerations

* Data transport in ROS 2 is non-deterministic. So is callback execution when multi-threaded executors are in place. This is true for ROS 2 code in general, and for process-wide APIs in particular, for which non-determinism is the price to pay for a synchronous programming model. As such, time sensitive and execution order dependent tests are bound to fail, even if only sporadically. Synchronization is necessary to avoid these issues, and fortunately the very same process-wide APIs enable safe use of synchronization primitives (e.g. via a multi-threaded executor spinning in a background thread).
* ROS 2 middlewares perform peer discovery by default. This allows distributed architectures in production but leads to cross-talk during parallelized testing. `domain_coordinator` functionality simplifies ROS domain ID assignment enforcing host-wide uniqueness and with it, middleware isolation.

## Rules of Thumb

1. Use `synchros2.scope.top` to setup ROS 2 in your test fixtures.
    * Isolate it by passing a unique domain ID, as provided by `domain_coordinator.domain_id`.
2. Use synchronization primitives to wait with timeouts.
    * Note timeouts make the test time sensitive. Pick timeouts an order of magnitude above the expected test timing.

## Writing integration tests using `pytest` (recommended)

[`pytest`](https://docs.pytest.org/en/7.4.x/) is a testing framework for Python software, the most common in ROS 2 Python codebases.

```python
import domain_coordinator
import pytest
from typing import Iterator
import synchros2.scope as ros_scope
from synchros2.scope import ROSAwareScope

@pytest.fixture
def ros() -> Iterator[ROSAwareScope]:
    """
    A pytest fixture that will set up and yield a ROS 2 aware global scope to each test that requests it.
    """
    with domain_coordinator.domain_id() as domain_id:
        with ros_scope.top(global_=True, namespace="fixture", domain_id=domain_id) as top:
            yield top

def test_it(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    ros.node.get_logger().info("Logging!")
```

## Writing integration tests using `unittest`

[`unittest`](https://docs.python.org/3/library/unittest.html) is the testing framework in Python's standard library.

```python
import contextlib
import domain_coordinator
import unittest
import synchros2.scope as ros_scope
from synchros2.scope import ROSAwareScope

class TestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = contextlib.ExitStack()
        domain_id = self.fixture.enter_context(domain_coordinator.domain_id())
        self.ros = self.fixture.enter_context(ros_scope.top(global_=True, namespace="fixture", domain_id=domain_id))
    def tearDown(self) -> None:
        self.fixture.close()
    def test_it(self) -> None:
        self.assertIsNotNone(self.ros.node)
        self.ros.node.get_logger().info("Logging!")
```

## Adding integration tests to a package

A package's type and build system dictate how unit tests are to be added. Unit tests for ROS 2 packages are typically hosted under the `test` subdirectory.

For `ament_cmake` packages, the `CMakeLists.txt` file should have:
```cmake
if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(unit_tests test)
endif()
```

For `ament_python` packages, the `setup.py` file should have:
```python
setup(
    # ...
    tests_require=['pytest'],
)
```

## Useful References

- [Official ROS 2 testing tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Testing-Main.html)
- [`pytest` documentation](https://pytest.org)
- [`unittest` documentation](https://docs.python.org/3/library/unittest.html)
