#  Copyright (c) 2023 Boston Dynamics AI Institute, Inc.  All rights reserved.
from threading import Event
from typing import Any, Optional

from rclpy.task import Future


def wait_until_future_complete(future: Future, timeout_sec: Optional[float] = None) -> Any:
    """Waits for a future assuming that the node which created the future is spinning somewhere else"""
    event = Event()

    def _done_callback(_: Any) -> None:
        event.set()

    future.add_done_callback(_done_callback)
    if not event.wait(timeout=timeout_sec):
        raise TimeoutError("Future did not complete in time")
    return future.result
