# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from typing import Callable, Optional, Type

import rclpy.action
from rclpy.node import Node

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.action_handle import ActionHandle
from bdai_ros2_wrappers.type_hints import Action


class ActionClientWrapper(rclpy.action.ActionClient):
    """A wrapper for ros2's ActionClient for extra functionality"""

    def __init__(self, action_type: Type[Action], action_name: str, node: Optional[Node] = None) -> None:
        """Constructor

        Args:
            action_type (Type[Action]): The type of the action
            action_name (str): The name of the action (for logging purposes)
            node (Optional[Node]): optional node for action client, defaults to the current process node
        """
        node = node or scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._node = node
        super().__init__(self._node, action_type, action_name)
        self._node.get_logger().info(f"Waiting for action server for {action_name}")
        self.wait_for_server()
        self._node.get_logger().info("Found server")

    def send_goal_and_wait(
        self, action_name: str, goal: Action.Goal, timeout_sec: Optional[float] = None
    ) -> Optional[Action.Result]:
        """Sends an action goal and waits for the result

        Args:
            action_name (str): A representative name of the action for logging
            goal (Action.Goal): The Action Goal sent to the action server
            timeout_sec (Optional[float]): A timeout for waiting on a response/result from the action server. Setting to
                None creates no timeout

        Returns:
            Optional[Action.Result]:
        """
        if goal is None:
            self._node.get_logger.warn("Cannot send NULL ActionGoal")
            return None
        self._node.get_logger().info(f"Sending Action [{action_name}]")

        canceled = False

        def _on_cancel_succeeded() -> None:
            nonlocal canceled
            canceled = True

        failed = False

        def _on_failure() -> None:
            nonlocal failed
            failed = True

        handle = self.send_goal_async_handle(action_name=action_name, goal=goal, on_failure_callback=_on_failure)
        handle.set_on_cancel_success_callback(_on_cancel_succeeded)
        if not handle.wait_for_result(timeout_sec=timeout_sec):
            # If the action didn't fail and wasn't canceled then it timed out and should be canceled
            if not failed and not canceled:
                handle.cancel()
                self._node.get_logger().error(f"Action [{action_name}] timed out")
            return None

        return handle.result

    def send_goal_async_handle(
        self,
        action_name: str,
        goal: Action.Goal,
        *,
        result_callback: Optional[Callable[[Action.Result], None]] = None,
        feedback_callback: Optional[Callable[[Action.Feedback], None]] = None,
        on_failure_callback: Optional[Callable[[], None]] = None,
    ) -> ActionHandle:
        """Sends an action goal asynchronously and create an ActionHandle for managing the asynchronous lifecycle of
            the action request

        Args:
            action_name (str): A representative name of the action for logging
            goal (Action.Goal): The Action Goal sent to the action server
            result_callback (Optional[Callable[[Action.Result], None]]): A callback to process/handle the resulting
                feedback from executing the command
            feedback_callback (Optional[Callable[[Action.Feedback], None]]): A callback to process/handle the feedback
                received during the execution of the command
            on_failure_callback (Optional[Callable[[None], None]]): A callback to process/handle when the action fails
                for various reasons

        Returns:
            ActionHandle: An object to manage the asynchronous lifecycle of the action request
        """
        handle = ActionHandle(action_name=action_name, logger=self._node.get_logger(), context=self._node.context)
        if result_callback is not None:
            handle.set_result_callback(result_callback)

        if on_failure_callback is not None:
            handle.set_on_failure_callback(on_failure_callback)

        if feedback_callback is None:
            send_goal_future = self.send_goal_async(goal)
        else:
            handle.set_feedback_callback(feedback_callback)
            send_goal_future = self.send_goal_async(goal, feedback_callback=handle.get_feedback_callback)
        handle.set_send_goal_future(send_goal_future)

        return handle
