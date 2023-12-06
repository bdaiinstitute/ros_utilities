# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import array
import time
from typing import Tuple

import pytest
from action_tutorials_interfaces.action import Fibonacci
from rclpy.action.server import GoalStatus, ServerGoalHandle

from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.single_goal_multiple_action_servers import (
    SingleGoalMultipleActionServers,
)


def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    """
    Executor callback for a server that does fibonacci
    """
    sequence = [0, 1]
    for i in range(1, goal_handle.request.order):
        sequence.append(sequence[i] + sequence[i - 1])

    goal_handle.succeed()

    result = Fibonacci.Result()
    result.sequence = sequence
    return result


def execute_callback_wrong_fib(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    """
    Different executor for another server that does fibonacci wrong
    """
    # time delay to make interrupting easier
    time.sleep(1)
    sequence = [0, 1]
    for i in range(1, goal_handle.request.order):
        sequence.append(sequence[i] * sequence[i - 1])

    result = None
    if goal_handle.status != GoalStatus.STATUS_ABORTED:
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
    else:
        result = Fibonacci.Result()
        result.sequence = [-1]

    return result


@pytest.fixture
def action_triplet(
    ros: ROSAwareScope,
) -> Tuple[SingleGoalMultipleActionServers, ActionClientWrapper, ActionClientWrapper]:
    action_parameters = [
        (Fibonacci, "fibonacci", execute_callback, None),
        (Fibonacci, "fibonacci_wrong", execute_callback_wrong_fib, None),
    ]
    assert ros.node is not None
    action_server = SingleGoalMultipleActionServers(ros.node, action_parameters)
    action_client_a = ActionClientWrapper(Fibonacci, "fibonacci", ros.node)
    action_client_b = ActionClientWrapper(Fibonacci, "fibonacci_wrong", ros.node)
    return action_server, action_client_a, action_client_b


def test_actions_in_sequence(
    action_triplet: Tuple[
        SingleGoalMultipleActionServers, ActionClientWrapper, ActionClientWrapper
    ]
) -> None:
    """
    Tests out normal operation with multiple action servers and clients
    """
    _, action_client_a, action_client_b = action_triplet
    goal = Fibonacci.Goal()
    goal.order = 5
    # use first client
    result = action_client_a.send_goal_and_wait(
        "action_request_a", goal=goal, timeout_sec=5
    )
    assert result is not None
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result
    # use second client
    result = action_client_b.send_goal_and_wait(
        "action_request_b", goal=goal, timeout_sec=5
    )
    assert result is not None
    expected_result = array.array("i", [0, 1, 0, 0, 0, 0])
    assert result.sequence == expected_result


def test_action_interruption(
    ros: ROSAwareScope,
    action_triplet: Tuple[
        SingleGoalMultipleActionServers, ActionClientWrapper, ActionClientWrapper
    ],
) -> None:
    """
    This test should start a delayed request from another client
    then make an immediate request to interrupt the last request.

    Due to the threading and reliance on sleeps this test might be
    tempermental on other machines.
    """
    _, action_client_a, action_client_b = action_triplet

    def deferred_request() -> None:
        # time delay to give other action time to get started before interrupting
        time.sleep(0.3)
        goal = Fibonacci.Goal()
        goal.order = 5
        action_client_a.send_goal_and_wait(
            "deferred_action_request", goal=goal, timeout_sec=2
        )

    assert ros.executor is not None
    ros.executor.create_task(deferred_request)

    # immediately start the request for other goal
    goal = Fibonacci.Goal()
    goal.order = 5
    result = action_client_b.send_goal_and_wait(
        "action_request", goal=goal, timeout_sec=5
    )
    assert result is None
