#  Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import Iterable

import domain_coordinator
import pytest

import synchros2.scope as scope
from synchros2.scope import ROSAwareScope


@pytest.fixture
def domain_id() -> Iterable[int]:
    with domain_coordinator.domain_id() as domain_id:  # to ensure node isolation
        yield domain_id


@pytest.fixture
def ros(domain_id: int) -> Iterable[ROSAwareScope]:
    with scope.top(global_=True, domain_id=domain_id, namespace="fixture") as top:
        yield top


@pytest.fixture
def verbose_ros(domain_id: int) -> Iterable[ROSAwareScope]:
    args = ["--ros-args", "--enable-rosout-logs", "--log-level", "INFO"]
    with scope.top(args, global_=True, domain_id=domain_id, namespace="fixture") as top:
        yield top
