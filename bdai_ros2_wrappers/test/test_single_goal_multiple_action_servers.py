# Copyright (c) 2023-2024 Boston Dynamics AI Institute Inc.  All rights reserved.
import array
from threading import Semaphore
from typing import Tuple

import pytest
from example_interfaces.action import Fibonacci
from rclpy.action.server import ServerGoalHandle

from bdai_ros2_wrappers.action import Actionable
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.single_goal_multiple_action_servers import SingleGoalMultipleActionServers


@pytest.fixture
def action_triplet(ros: ROSAwareScope) -> Tuple[Semaphore, Actionable, Actionable]:
    semaphore = Semaphore(0)

    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        nonlocal semaphore
        semaphore.acquire()

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])

        result = Fibonacci.Result()
        if not goal_handle.is_cancel_requested:
            result.sequence = sequence
            goal_handle.succeed()
        else:
            goal_handle.canceled()
        return result

    def reversed_execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        nonlocal semaphore
        semaphore.acquire()

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])

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
    SingleGoalMultipleActionServers(ros.node, action_parameters)
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    compute_fibonacci_reversed = Actionable(Fibonacci, "fibonacci/compute_reversed", ros.node)
    return semaphore, compute_fibonacci, compute_fibonacci_reversed


def test_actions_in_sequence(
    action_triplet: Tuple[Semaphore, Actionable, Actionable],
) -> None:
    semaphore, compute_fibonacci, compute_fibonacci_reversed = action_triplet

    semaphore.release()  # allow for action A
    semaphore.release()  # allow for action B

    goal = Fibonacci.Goal()
    goal.order = 5
    result = compute_fibonacci(goal)
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result
    result = compute_fibonacci_reversed(goal)
    expected_result = array.array("i", [5, 3, 2, 1, 1, 0])
    assert result.sequence == expected_result


def test_same_action_interruption(
    action_triplet: Tuple[Semaphore, Actionable, Actionable],
) -> None:
    semaphore, compute_fibonacci, _ = action_triplet

    goal = Fibonacci.Goal()
    goal.order = 5
    action_a = compute_fibonacci.asynchronously(goal)
    action_b = compute_fibonacci.asynchronously(goal)
    semaphore.release()  # allow for action A
    semaphore.release()  # allow for action B
    assert wait_for_future(action_a.finalization, timeout_sec=5.0)
    assert action_a.cancelled
    assert wait_for_future(action_b.finalization, timeout_sec=5.0)
    assert action_b.succeeded
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action_b.result.sequence == expected_result


def test_different_action_interruption(
    action_triplet: Tuple[Semaphore, Actionable, Actionable],
) -> None:
    semaphore, compute_fibonacci, compute_fibonacci_reversed = action_triplet

    goal = Fibonacci.Goal()
    goal.order = 5
    action_a = compute_fibonacci.asynchronously(goal)
    action_b = compute_fibonacci_reversed.asynchronously(goal)
    semaphore.release()  # allow for action A
    semaphore.release()  # allow for action B
    assert wait_for_future(action_a.finalization, timeout_sec=5.0)
    assert action_a.cancelled
    assert wait_for_future(action_b.finalization, timeout_sec=5.0)
    assert action_b.succeeded
    expected_result = array.array("i", [5, 3, 2, 1, 1, 0])
    assert action_b.result.sequence == expected_result
