# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from threading import Event
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from rclpy.action.client import ClientGoalHandle
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.task import Future

from bdai_ros2_wrappers.type_hints import Action


class ActionHandle(object):
    """Handles the two stage process that is getting a result after sending an ActionGoal to an ActionServer as well
    as holding the various callbacks for sending an ActionGoal (cancel, failure, feedback, result)
    """

    def __init__(self, action_name: str, logger: Optional[RcutilsLogger] = None):
        """Constructor

        Args:
            action_name (str): The name of the action (for logging purposes)
            logger (Optional[RcutilsLogger]): An optional logger to use. If none is provided, one is created
        """
        self._action_name = action_name
        self._send_goal_future: Optional[Future] = None
        self._goal_handle: Optional[ClientGoalHandle] = None
        self._get_result_future: Optional[Future] = None
        self._feedback_callback: Optional[Callable[[Action.Feedback], None]] = None
        self._wait_for_result_callback: Optional[Callable[[], None]] = None
        self._result_callback: Optional[Callable[[Action.Result], None]] = None
        self._on_failure_callback: Optional[Callable] = None
        self._cancel_future: Optional[Future] = None
        self._on_cancel_success_callback: Optional[Callable] = None
        self._on_cancel_failure_callback: Optional[Callable] = None
        self._result: Optional[Action.Result] = None
        if logger is None:
            self._logger = RcutilsLogger(name=f"{action_name} Handle")
        else:
            self._logger = logger

    @property
    def result(self) -> Optional[Action.Result]:
        """Returns the result of the action if it has been received from the ActionServer, None otherwise"""
        return self._result

    def wait_for_result(self, timeout_sec: Optional[float] = None) -> bool:
        """Waits until a result is received or times out

        Args:
            timeout_sec (Optional[float]): A timeout in seconds. No timeout is used if None

        Returns:
            True if successful, False if the timeout was triggered
        """
        event = Event()

        def done_callback() -> None:
            nonlocal event
            event.set()

        self._wait_for_result_callback = done_callback

        return event.wait(timeout=timeout_sec)

    def set_send_goal_future(self, send_goal_future: Future) -> None:
        """Sets the future received from sending the Action.Goal and sets up the callback for when a response is
        received"""
        self._send_goal_future = send_goal_future
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def set_feedback_callback(self, feedback_callback: Callable[[Action.Feedback], None]) -> None:
        """Sets the callback used to process feedback received while an Action is being executed"""
        self._feedback_callback = feedback_callback

    def set_result_callback(self, result_callback: Callable[[Action.Result], None]) -> None:
        """Sets the callback for processing the result from executing an Action"""
        self._result_callback = result_callback

    def set_on_failure_callback(self, on_failure_callback: Callable) -> None:
        """Set the callback to execute upon failure"""
        self._on_failure_callback = on_failure_callback

    def set_on_cancel_success_callback(self, on_cancel_success_callback: Callable) -> None:
        """Set the callback to execute upon successfully canceling the action"""
        self._on_cancel_success_callback = on_cancel_success_callback

    def set_on_cancel_failure_callback(self, on_cancel_failure_callback: Callable) -> None:
        """Set the callback to execute upon failing to cancel the action"""
        self._on_cancel_failure_callback = on_cancel_failure_callback

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

    def get_feedback_callback(self, feedback: Action.Feedback) -> None:
        """Public feedback callback that can be given to the ActionClient

        Currently just passes the feedback to the user provided callback
        """
        if self._feedback_callback is not None:
            self._feedback_callback(feedback)

    def _get_result_callback(self, future: Future) -> None:
        """Callback for pulling the resulting feedback out of the future and passing it to a user provided callback"""
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._logger.info("Finished successfully")
            self._result = result.result
            if self._wait_for_result_callback is not None:
                self._wait_for_result_callback()
            if self._result_callback is not None:
                self._result_callback(self._result)
            return
        elif result.status == GoalStatus.STATUS_ABORTED:
            self._logger.info("Aborted")
            self._failure()
            return
        elif result.status == GoalStatus.STATUS_CANCELED:
            self._logger.info("Canceled")
            if self._on_cancel_success_callback is not None:
                self._on_cancel_success_callback()
            return

    def cancel(self) -> None:
        """Send the desire to cancel a command"""
        self._logger.info("Canceling Action")

        if self._goal_handle is not None:
            self._cancel_future = self._goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future: Future) -> None:
        """Callback for handling the response to attempting to cancel a command"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) == 0:
            self._logger.info("Cancel request rejected")
            if self._on_cancel_failure_callback is not None:
                self._on_cancel_failure_callback()
        else:
            self._logger.info("Cancel request accepted")

    def _failure(self) -> None:
        """Triggers the internal failure callback if it exists"""
        if self._on_failure_callback is not None:
            self._on_failure_callback()
