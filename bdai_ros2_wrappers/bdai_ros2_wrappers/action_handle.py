# Copyright (c) 2023 Boston Dynamics AI Institute, Inc. All rights reserved.
from threading import Event
from typing import Optional, Callable, TypeVar
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle
from rclpy.impl.rcutils_logger import RcutilsLogger


# The specific ActionType is created by the .action file
ActionType = TypeVar("ActionType")


def wait_for_future(future: Future, timeout_sec: Optional[float] = None) -> bool:
    """Blocks while waiting for a future to become done

    Args:
        future: The future to be waited on
        timeout_sec: An optional timeout for who long to wait

    Returns:
        True if successful, False if the timeout was triggered
    """
    event = Event()

    def done_callback(_):
        nonlocal event
        event.set()

    future.add_done_callback(done_callback)
    if timeout_sec is None:
        return event.wait()
    else:
        return event.wait(timeout_sec)


class ActionHandle(object):
    def __init__(self, action_name: str, logger: Optional[RcutilsLogger] = None):
        self._action_name = action_name
        self._send_goal_future: Optional[Future] = None
        self._goal_handle: Optional[ClientGoalHandle] = None
        self._get_result_future: Optional[Future] = None
        self._feedback_callback_internal: Optional[
            Callable[[ActionType.Feedback], None]
        ] = None
        self._result_callback_internal: Optional[
            Callable[[ActionType.Feedback], None]
        ] = None
        self._on_failure_callback: Optional[Callable] = None
        self.result: Optional[ActionType.Feedback] = None
        if logger is None:
            self._logger = RcutilsLogger(name=f"{action_name} Handle")
        else:
            self._logger = logger

    def set_send_goal_future(self, send_goal_future: Future) -> Future:
        """Sets the future received from sending the Action.Goal and sets up the callback for when a response is
        received"""
        self._send_goal_future = send_goal_future
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def set_feedback_callback(
        self, feedback_callback: Callable[[ActionType.Feedback], None]
    ) -> None:
        """Sets the callback used to process feedback received while an Action is being executed"""
        self._feedback_callback_internal = feedback_callback

    def set_result_callback(
        self, result_callback: Callable[[ActionType.Feedback], None]
    ) -> None:
        """Sets the callback for processing the result from executing an Action"""
        self._result_callback_internal = result_callback

    def set_on_failure_callback(self, on_failure_callback: Callable) -> None:
        """Set the callback to execute upon failure"""
        self._on_failure_callback = on_failure_callback

    def _goal_response_callback(self, future: Future) -> None:
        """Callback that handles receiving a response from the ActionServer

        Note: This does not handle receiving a result directly
        """
        if future.result() is None:
            self._logger.error(f"Action request failed: {future.exception():!r}")
            self._failure()
            return
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._logger.error("Action rejected")
            self._failure()
            return

        self._logger.info("Action accepted")

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def get_feedback_callback(self, feedback: ActionType.Feedback) -> None:
        """Public feedback callback that can be given to the ActionClient

        Currently just passes the feedback to the user provided callback
        """
        if self._feedback_callback_internal is not None:
            self._feedback_callback_internal(feedback.feedback)

    def _get_result_callback(self, future: Future) -> None:
        """Callback for pulling the resulting feedback out of the future and passing it to a user provided callback"""
        result = future.result().result
        if not result.success:
            self._logger.error(f"Action resulted in failure. Msg: {result.message}")
            self._failure()
            return

        self.result = result.result
        if self._result_callback_internal is not None:
            self._result_callback_internal(self.result)

    def cancel(self) -> None:
        """Send the desire to cancel a command"""
        self._logger.debug("Canceling RobotCommand")

        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future: Future) -> None:
        """Callback for handling the response to attempting to cancel a command"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self._logger.debug("Action successfully canceled")
            # TODO: callback?
        else:
            self._logger.debug("Action failed to cancel")
            # TODO: callback?

    def _failure(self) -> None:
        """Triggers the internal failure callback if it exists"""
        if self._on_failure_callback is not None:
            self._on_failure_callback()
