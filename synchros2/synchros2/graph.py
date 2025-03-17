#  Copyright (c) 2025 Boston Dynamics AI Institute LLC.  All rights reserved.
import functools
from typing import Callable, Optional

from rclpy.node import Node
from rclpy.task import Future

from synchros2.futures import unwrap_future


def _ensure_num_matched_async(
    node: Node,
    lower_bound: int,
    num_matched_func: Callable[[], int],
    *,
    poll_period_sec: float = 0.1,
) -> Future:
    """Returns a future to the number of matched elements based a user-provided function.

    Note this method relies on polling. It is frequently used to count the number of publishers or subscriptions on a
    topic and in ROS 2 Humble and earlier distributions matching events are missing.

    Args:
        node: A ROS 2 node
        lower_bound: The lower bound on the number of matched elements
        num_matched_func: The function to use to count the number of matched elements
        poll_period_sec: The period for polling the count

    Returns:
        A future to the number of matched elements
    """
    future = Future()
    num_matched = num_matched_func()
    if num_matched < lower_bound:

        def _poll() -> None:
            nonlocal future, lower_bound
            if future.cancelled():
                return
            num_matched: int = num_matched_func()
            assert num_matched is not None
            if lower_bound <= num_matched:
                future.set_result(num_matched)

        timer = node.create_timer(poll_period_sec, _poll)
        future.add_done_callback(lambda _: node.destroy_timer(timer))
    else:
        future.set_result(num_matched)
    return future


def _ensure_num_matched(
    node: Node,
    lower_bound: int,
    num_matched_func: Callable[[], int],
    timeout_sec: Optional[float] = None,
    *,
    poll_period_sec: float = 0.1,
) -> int:
    """Returns the number of matched elements based a user-provided function.

    Note this method relies on polling. It is frequently used to count the number of publishers or subscriptions on a
    topic and in ROS 2 Humble and earlier distributions matching events are missing.

    Args:
        node: A ROS 2 node
        lower_bound: The lower bound on the number of matched elements
        num_matched_func: The function to use to count the number of matched elements
        timeout_sec: A timeout on how long to wait for the lower bound to be satisfied
        poll_period_sec: The period for polling the count

    Returns:
        The number of matched elements

    Exceptions:
        TimeoutError if the lower bound has not been satisfied in `timeout_sec` seconds.
    """
    future = _ensure_num_matched_async(
        node=node,
        lower_bound=lower_bound,
        num_matched_func=num_matched_func,
        poll_period_sec=poll_period_sec,
    )
    return unwrap_future(future, timeout_sec=timeout_sec)


def ensure_num_publishers_async(
    node: Node,
    topic_name: str,
    num_publishers: int,
    *,
    poll_period_sec: float = 0.1,
) -> Future:
    """Returns a future to the number of publishers on a topic.

    Note that in ROS 2 Humble and earlier distributions, this method relies on
    polling the number of known publishers for the topic subscribed, as subscription
    matching events are missing.

    Args:
        node: A ROS 2 node
        topic_name: The name of the topic to check
        num_publishers: The lower bound on the number of publishers to match.
        timeout_sec: A timeout on how long to wait for the lower bound to be satisfied
        poll_period_sec: The period for polling the count

    Returns:
        A future to the number of publishers
    """
    return _ensure_num_matched_async(
        node=node,
        lower_bound=num_publishers,
        num_matched_func=functools.partial(node.count_publishers, topic_name),
        poll_period_sec=poll_period_sec,
    )


def ensure_num_publishers(
    node: Node,
    topic_name: str,
    num_publishers: int,
    timeout_sec: Optional[float] = None,
    *,
    poll_period_sec: float = 0.1,
) -> int:
    """Returns the number of publishers on a topic.

    Note that in ROS 2 Humble and earlier distributions, this method relies on
    polling the number of known publishers for the topic subscribed, as subscription
    matching events are missing.

    Args:
        node: A ROS 2 node
        topic_name: The name of the topic to check
        num_publishers: The lower bound on the number of publishers to match.
        timeout_sec: A timeout on how long to wait for the lower bound to be satisfied
        poll_period_sec: The period for polling the count

    Returns:
        The number of publishers
    """
    return _ensure_num_matched(
        node=node,
        lower_bound=num_publishers,
        num_matched_func=functools.partial(node.count_publishers, topic_name),
        timeout_sec=timeout_sec,
        poll_period_sec=poll_period_sec,
    )


def ensure_num_subscriptions_async(
    node: Node,
    topic_name: str,
    num_subscriptions: int,
    *,
    poll_period_sec: float = 0.1,
) -> Future:
    """Returns a future to the number of subscriptions on a topic.

    Note that in ROS 2 Humble and earlier distributions, this method relies on
    polling the number of known subscriptions for the topic subscribed, as subscription
    matching events are missing.

    Args:
        node: A ROS 2 node
        topic_name: The name of the topic to check
        num_subscriptions: The lower bound on the number of subscriptions to match.
        timeout_sec: A timeout on how long to wait for the lower bound to be satisfied
        poll_period_sec: The period for polling the count

    Returns:
        A future to the number of publishers
    """
    return _ensure_num_matched_async(
        node=node,
        lower_bound=num_subscriptions,
        num_matched_func=functools.partial(node.count_subscribers, topic_name),
        poll_period_sec=poll_period_sec,
    )


def ensure_num_subscriptions(
    node: Node,
    topic_name: str,
    num_subscriptions: int,
    timeout_sec: Optional[float] = None,
    *,
    poll_period_sec: float = 0.1,
) -> int:
    """Returns the number of subscriptions on a topic.

    Note that in ROS 2 Humble and earlier distributions, this method relies on
    polling the number of known subscriptions for the topic subscribed, as subscription
    matching events are missing.

    Args:
        node: A ROS 2 node
        topic_name: The name of the topic to check
        num_subscriptions: The lower bound on the number of subscriptions to match.
        timeout_sec: A timeout on how long to wait for the lower bound to be satisfied
        poll_period_sec: The period for polling the count

    Returns:
        The number of subscriptions
    """
    return _ensure_num_matched(
        node=node,
        lower_bound=num_subscriptions,
        num_matched_func=functools.partial(node.count_subscribers, topic_name),
        timeout_sec=timeout_sec,
        poll_period_sec=poll_period_sec,
    )
