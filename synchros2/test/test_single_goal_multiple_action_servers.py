# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC.  All rights reserved.
import array
from threading import Barrier, Lock
from typing import Iterable, Tuple

import pytest
from example_interfaces.action import Fibonacci
from rclpy.action.server import ServerGoalHandle
from typing_extensions import TypeAlias

from synchros2.action import Actionable
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope
from synchros2.single_goal_multiple_action_servers import SingleGoalMultipleActionServers


@pytest.fixture
def action_triplet(ros: ROSAwareScope) -> Iterable[Tuple[Barrier, Actionable, Actionable]]:
    lock = Lock()
    barrier = Barrier(2)

    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        nonlocal barrier, lock

        if not barrier.broken:
            barrier.wait()

        with lock:
            sequence = [0, 1]
            for i in range(1, goal_handle.request.order):
                sequence.append(sequence[i] + sequence[i - 1])

            if not barrier.broken:
                barrier.wait()

            result = Fibonacci.Result()
            if not goal_handle.is_cancel_requested:
                result.sequence = sequence
                goal_handle.succeed()
            else:
                goal_handle.canceled()
            return result

    def reversed_execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        nonlocal barrier, lock

        if not barrier.broken:
            barrier.wait()

        with lock:
            sequence = [0, 1]
            for i in range(1, goal_handle.request.order):
                sequence.append(sequence[i] + sequence[i - 1])

            if not barrier.broken:
                barrier.wait()

            result = Fibonacci.Result()
            if not goal_handle.is_cancel_requested:
                result.sequence = list(reversed(sequence))
                goal_handle.succeed()
            else:
                goal_handle.canceled()
            return result

    action_parameters = [
        (Fibonacci, "fibonacci/compute", execute_callback, None),
        (Fibonacci, "fibonacci/compute_reversed", reversed_execute_callback, None),
    ]
    assert ros.node is not None
    SingleGoalMultipleActionServers(ros.node, action_parameters, nosync=True)
    FibonacciActionable: TypeAlias = Actionable[Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback]
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    compute_fibonacci_reversed: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute_reversed", ros.node)

    try:
        yield barrier, compute_fibonacci, compute_fibonacci_reversed
    finally:
        barrier.abort()


def test_actions_in_sequence(
    action_triplet: Tuple[Barrier, Actionable, Actionable],
) -> None:
    barrier, compute_fibonacci, compute_fibonacci_reversed = action_triplet

    barrier.abort()  # avoid synchronization

    goal = Fibonacci.Goal()
    goal.order = 5
    result = compute_fibonacci(goal)
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result
    result = compute_fibonacci_reversed(goal)
    expected_result = array.array("i", [5, 3, 2, 1, 1, 0])
    assert result.sequence == expected_result


def test_same_action_interruption(
    action_triplet: Tuple[Barrier, Actionable, Actionable],
) -> None:
    barrier, compute_fibonacci, _ = action_triplet

    goal = Fibonacci.Goal()
    goal.order = 5
    action_a = compute_fibonacci.asynchronously(goal)
    barrier.wait(timeout=5.0)  # let action A start
    action_b = compute_fibonacci.asynchronously(goal)
    # Actions B and A will allow each other to start and finish, respectively
    assert wait_for_future(action_a.finalization, timeout_sec=5.0)
    assert action_a.cancelled
    barrier.wait(timeout=5.0)  # let action B finish
    assert wait_for_future(action_b.finalization, timeout_sec=5.0)
    assert action_b.succeeded
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action_b.result.sequence == expected_result


def test_different_action_interruption(
    action_triplet: Tuple[Barrier, Actionable, Actionable],
) -> None:
    barrier, compute_fibonacci, compute_fibonacci_reversed = action_triplet

    goal = Fibonacci.Goal()
    goal.order = 5
    action_a = compute_fibonacci.asynchronously(goal)
    barrier.wait(timeout=5.0)  # let action A start
    action_b = compute_fibonacci_reversed.asynchronously(goal)
    # Actions B and A will allow each other to start and finish, respectively
    assert wait_for_future(action_a.finalization, timeout_sec=5.0)
    assert action_a.cancelled
    barrier.wait(timeout=5.0)  # let action B finish
    assert wait_for_future(action_b.finalization, timeout_sec=5.0)
    assert action_b.succeeded
    expected_result = array.array("i", [5, 3, 2, 1, 1, 0])
    assert action_b.result.sequence == expected_result
