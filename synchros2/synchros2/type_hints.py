# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
from typing import TypeVar

# The specific Action is created by the .action file
Action = TypeVar("Action")
Action_Goal = TypeVar("Action_Goal")
Action_Feedback = TypeVar("Action_Feedback")
Action_Result = TypeVar("Action_Result")

# Use for type hints where you are passing the type of an action, not an action object
ActionType = TypeVar("ActionType")

# The actual code generated for actions does something like this
Action.Goal = Action_Goal  # type: ignore
Action.Feedback = Action_Feedback  # type: ignore
Action.Result = Action_Result  # type: ignore

# The specific Msg is created by the .msg file
Msg = TypeVar("Msg")

# The specific Service is created by the .srv file
Srv = TypeVar("Srv")
SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
