# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

import inspect
from collections.abc import Sequence
from typing import Any, Callable, Iterator, Optional, Type, Union

import action_msgs.msg
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.utilities import Tape


class ActionException(Exception):
    """Base action exception."""

    def __init__(self, action: "ActionFuture") -> None:
        super().__init__(action)
        self.action = action


class ActionTimeout(ActionException):
    """Exception raised on action timeout."""

    pass


class ActionRejected(ActionException):
    """Exception raised when the action is rejected."""

    pass


class ActionAborted(ActionException):
    """Exception raised when the action is aborted."""

    pass


class ActionCancelled(ActionException):
    """Exception raised when the action is cancelled."""

    pass


class ActionFuture:
    """A proxy to a ROS 2 action invocation.

    Action futures are to actions what plain futures are to services, with a bit more functionality
    to cover action specific semantics such as feedback and cancellation.

    Action futures are rarely instantiated explicitly, but implicitly through `Actionable` APIs.
    """

    def __init__(self, goal_handle_future: Future, feedback_tape: Optional[Tape] = None) -> None:
        """Initializes the action future.

        Args:
            goal_handle_future: action goal handle future, as returned by the
            `rclpy.action.ActionClient.send_goal_async` method.
            feedback_tape: optional feedback tape, assumed to be tracking action feedback.
        """
        self._executor_ref = goal_handle_future._executor
        self._goal_handle_future = goal_handle_future
        if feedback_tape is not None:
            executor = self._executor_ref()
            if executor is not None:
                executor.context.on_shutdown(feedback_tape.close)
        self._feedback_tape = feedback_tape
        self._result_future: Optional[Future] = None
        self._acknowledgement_future = Future(executor=self._executor_ref())
        self._finalization_future = Future(executor=self._executor_ref())
        self._goal_handle_future.add_done_callback(self._goal_handle_callback)

    def _goal_handle_callback(self, goal_handle_future: Future) -> None:
        """Callback on action goal handle future resolution."""
        exception = goal_handle_future.exception()
        if exception is not None:
            self._acknowledgement_future.set_exception(exception)
            # An exception during acknowledgement is a rejection
            self._finalization_future.set_result(None)
            return
        goal_handle = goal_handle_future.result()
        self._acknowledgement_future.set_result(goal_handle.accepted)
        if goal_handle.accepted:
            self._result_future = goal_handle.get_result_async()
            self._result_future.add_done_callback(self._result_callback)
        else:
            self._finalization_future.set_result(None)

    def _result_callback(self, result_future: Future) -> None:
        """Callback on action result future resolution."""
        exception = result_future.exception()
        if exception is None:
            self._finalization_future.set_result(result_future.result().status)
        else:
            self._finalization_future.set_exception(exception)
        if self._feedback_tape is not None:
            self._feedback_tape.close()

    @property
    def acknowledgement(self) -> Future:
        """Get future to wait for action acknowledgement (either acceptance or rejection)."""
        return self._acknowledgement_future

    @property
    def acknowledged(self) -> bool:
        """Check if action was acknowledged (i.e. accepted or rejected)"""
        return self._acknowledgement_future.done()

    @property
    def goal_handle(self) -> ClientGoalHandle:
        """Get action goal handle.

        Raises:
            RuntimeError: if action has not been accepted or rejected yet.
        """
        if not self._goal_handle_future.done():
            raise RuntimeError("Action not acknowledged yet")
        return self._goal_handle_future.result()

    @property
    def finalization(self) -> Future:
        """Get future to wait for action finalization.

        Action rejection also counts as finalization.
        """
        return self._finalization_future

    @property
    def finalized(self) -> bool:
        """Check if action was finalized."""
        return self._finalization_future.done()

    @property
    def result(self) -> Any:
        """Get action result.

        Raises:
            RuntimeError: if action is still executing.
        """
        if not self.acknowledged:
            raise RuntimeError("Action not acknowledged")
        if not self.accepted:
            raise RuntimeError("Action not accepted")
        if self._result_future is None or not self._result_future.done():
            raise RuntimeError("Action still executing")
        return self._result_future.result().result

    @property
    def tracks_feedback(self) -> bool:
        """Check if action future tracks action feedback."""
        return self._feedback_tape is not None

    @property
    def feedback(self) -> Sequence:
        """Get all buffered action feedback.

        Action must have been accepted before any feedback can be fetched.

        Raises:
            RuntimeError: if action feedback tracking is not enabled,
            action acknowledgement has not been received yet, or
            action was not accepted.
        """
        if self._feedback_tape is None:
            raise RuntimeError("Action feedback tracking is disabled")
        if not self._goal_handle_future.done():
            raise RuntimeError("Action not acknowledged yet")
        goal_handle = self._goal_handle_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Action not accepted")
        return list(self._feedback_tape.content())

    def feedback_stream(
        self,
        *,
        forward_only: bool = False,
        buffer_size: Optional[int] = None,
        timeout_sec: Optional[float] = None,
    ) -> Iterator:
        """Iterate over action feedback as it comes.

        Iteration stops when the given timeout expires or when the action is done executing.
        Note that iterating over action feedback to come is a blocking operation.

        Action must have been accepted before feedback streaming is allowed.

        Args:
            forward_only: whether to ignore buffered action feedback or not.
            buffer_size: optional maximum size for the incoming feedback buffer.
            If none is provided, the buffer will grow unbounded.
            timeout_sec: optional timeout, in seconds, for new action feedback.

        Returns:
            a lazy iterator over action feedback.

        Raises:
            RuntimeError: if action feedback tracking is not enabled,
            action acknowledgement has not been received yet, or action
            was not accepted.
        """
        if self._feedback_tape is None:
            raise RuntimeError("Action feedback tracking is not enabled")
        if not self._goal_handle_future.done():
            raise RuntimeError("Action not acknowledged yet")
        goal_handle = self._goal_handle_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Action not accepted")
        outerframe = inspect.stack()[1]
        return self._feedback_tape.content(
            follow=True,
            forward_only=forward_only,
            buffer_size=buffer_size,
            timeout_sec=timeout_sec,
            label=f"{outerframe.filename}:{outerframe.lineno}",
        )

    @property
    def accepted(self) -> bool:
        """Check if action was accepted."""
        return not self._goal_handle_future.done() or self._goal_handle_future.result().accepted

    @property
    def cancelled(self) -> bool:
        """Check if action was cancelled."""
        return (
            self._result_future is not None
            and self._result_future.done()
            and (self._result_future.result().status == action_msgs.msg.GoalStatus.STATUS_CANCELED)
        )

    @property
    def aborted(self) -> bool:
        """Check if action was aborted."""
        return (
            self._result_future is not None
            and self._result_future.done()
            and (self._result_future.result().status == action_msgs.msg.GoalStatus.STATUS_ABORTED)
        )

    @property
    def succeeded(self) -> bool:
        """Check if action was succeeded."""
        return (
            self._result_future is not None
            and self._result_future.done()
            and (self._result_future.result().status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED)
        )

    @property
    def status(self) -> Optional[int]:
        """Get action status code.

        See `action_msgs.msg.GoalStatus` documentation for status codes.
        Rejected actions are conventially assigned `None` for status.

        Raises:
            RuntimeError: if action has not been acknowledged or is still executing.
        """
        if not self._goal_handle_future.done():
            raise RuntimeError("Action not acknowledged yet")
        if not self._goal_handle_future.result().accepted:
            return None
        if self._result_future is None or not self._result_future.done():
            raise RuntimeError("Action still executing")
        return self._result_future.result().status

    def as_future(self) -> Future:
        """Get action future as a plain future.

        Useful to pass action futures to APIs that expect plain futures.
        """
        future = Future(executor=self._executor_ref())

        def _done_callback(_: Future) -> None:
            nonlocal future
            if future.cancelled():
                self.cancel()

        future.add_done_callback(_done_callback)

        def _bridge_callback(finalization_future: Future) -> None:
            nonlocal future
            if not self.accepted:
                future.set_exception(ActionRejected(self))
                return
            if self.aborted:
                future.set_exception(ActionRejected(self))
                return
            if self.cancelled:
                future.set_exception(ActionCancelled(self))
                return
            exception = finalization_future.exception()
            if exception:
                future.set_exception(exception)
                return
            future.set_result(self.result)

        self._finalization_future.add_done_callback(_bridge_callback)

        return future

    def cancel(self) -> Future:
        """Cancel action.

        Returns:
            a future for the cancellation status code.
            See `action_msgs.srv.CancelGoal` documentation for
            the list of possible status codes.
        """
        cancellation_future = Future(executor=self._executor_ref())

        def _response_callback(cancel_goal_future: Future) -> None:
            exception = cancel_goal_future.exception()
            if exception is None:
                response = cancel_goal_future.result()
                cancellation_future.set_result(response.return_code)
            else:
                cancellation_future.set_exception(exception)

        cancel_goal_future = self.goal_handle.cancel_goal_async()
        cancel_goal_future.add_done_callback(_response_callback)
        return cancellation_future


class Actionable:
    """An ergonomic interface to call actions in ROS 2.

    Actionables wrap `rclpy.action.ActionClient` instances to allow for synchronous
    and asynchronous action invocation, in a way that resembles remote procedure calls.
    """

    def __init__(self, action_type: Type, action_name: str, node: Optional[Node] = None, **kwargs: Any) -> None:
        """Initializes the actionable.

        Args:
            action_type: Target action type class (as generated by ``rosidl``).
            action_name: Name of the target action in the ROS 2 graph.
            node: optional node for the underlying action client, defaults to
            the current process node.
            kwargs: other keyword arguments are forwarded to the underlying action client.
            See `rclpy.action.ActionClient` documentation for further reference.
        """
        node = node or scope.node()
        self._action_type = action_type
        self._action_name = action_name
        self._action_client = ActionClient(node, action_type, action_name, **kwargs)

    @property
    def action_name(self) -> str:
        """Get the target action name."""
        return self._action_name

    @property
    def action_type(self) -> Type:
        """Get the target action type."""
        return self._action_type

    @property
    def action_client(self) -> ActionClient:
        """Get the underlying action client."""
        return self._action_client

    def wait_for_server(self, *args: Any, **kwargs: Any) -> bool:
        """Wait for action server to become available.

        See `rclpy.action.ActionClient.wait_for_server()`
        documentation for further reference.
        """
        return self._action_client.wait_for_server(*args, **kwargs)

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        """Forward invocation to `Actionable.synchronously`."""
        return self.synchronously(*args, **kwargs)

    def synchronously(
        self,
        goal: Any,
        *,
        feedback_callback: Optional[Callable] = None,
        timeout_sec: Optional[float] = None,
        nothrow: bool = False,
    ) -> Any:
        """Invoke action synchronously.

        Args:
            goal: goal to invoke action with.
            feedback_callback: optional action feedback callback.
            timeout_sec: optional action timeout, in seconds. If a timeout is specified and it
            expires, the action will be cancelled and the call will raise. Note this timeout is
            local to the caller. It may take some time for the action to be cancelled, that is
            if cancellation is not rejected by the action server.
            nothrow: if set to true, errors will not raise exceptions.

        Returns:
            the action result or None on error if `nothrow` was set and no result is available.

        Raises:
            ActionTimeout: if the action timed out and `nothrow` was not set.
            ActionRejected: if the action was not accepted and `nothrow` was not set.
            ActionCancelled: if the action was cancelled and `nothrow` was not set.
            ActionAborted: if the action was aborted and `nothrow` was not set.
            RuntimeError: if there is an internal server error and `nothrow` was not set.
        """
        action = ActionFuture(self._action_client.send_goal_async(goal, feedback_callback))
        if not wait_for_future(action.finalization, timeout_sec=timeout_sec):
            action.cancel()  # proactively cancel
            if nothrow:
                return None
            raise ActionTimeout(action)
        if not action.accepted:
            if nothrow:
                return None
            raise ActionRejected(action)
        if not nothrow:
            if action.cancelled:
                raise ActionCancelled(action)
            if action.aborted:
                raise ActionAborted(action)
            if not action.succeeded:
                raise RuntimeError("internal server error")
        return action.result

    def asynchronously(self, goal: Any, *, track_feedback: Union[int, bool] = False) -> ActionFuture:
        """Invoke action asynchronously.

        Args:
            goal: goal to invoke action with.
            track_feedback: whether and how to track action feedback. Other
            than a boolean to enable or disable tracking, a positive integer
            may be provided to cap feedback buffer size.

        Returns:
            the action future.
        """
        feedback_tape: Optional[Tape] = None
        if track_feedback is not False:
            feedback_tape_length = None
            if track_feedback is not True:
                feedback_tape_length = track_feedback
            feedback_tape = Tape(feedback_tape_length)
            goal_handle_future = self._action_client.send_goal_async(
                goal,
                lambda feedback: feedback_tape.write(feedback.feedback),
            )
        else:
            goal_handle_future = self._action_client.send_goal_async(goal)
        return ActionFuture(goal_handle_future, feedback_tape)
