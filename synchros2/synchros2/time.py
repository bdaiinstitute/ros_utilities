# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

from datetime import datetime, timedelta
from typing import Union

from rclpy.duration import Duration
from rclpy.time import Time


def as_proper_time(time: Union[int, float, datetime, Time]) -> Time:
    """Convert `time` to a proper, standardized `Time` object.

    For conversion, the following rules apply:
    - if an `int` or a `float` is provided, it is assumed to be a timestamp expressed in seconds.
    - if a `datetime` is provided, its UTC `datetime.timestamp()` is used.
    - if a `Duration` is provided, it is returned as is.

    Args:
        time: the time to be converted.

    Returns:
        an standardized `Time` object in ROS time, representing the given `time`.

    Raises:
        ValueError: if the given `time` is not of a supported type.
    """
    if isinstance(time, (int, float)):
        return Time(seconds=time)
    if isinstance(time, datetime):
        return Time(seconds=time.timestamp())
    if not isinstance(time, Time):
        raise ValueError(f"unsupported time type: {time}")
    return time


def as_proper_duration(duration: Union[int, float, timedelta, Duration]) -> Duration:
    """Convert `duration` to a proper, standardized `Duration` object.

    For conversion, the following rules apply:
    - if an `int` or a `float` is provided, it is assumed to be a duration expressed in seconds.
    - if a `timedelta` is provided, `timedelta.total_seconds()` are used.
    - if a `Duration` is provided, it is returned as is.

    Args:
        duration: the duration to be converted.

    Returns:
        an standardized `Duration` object representing the given `duration`.

    Raises:
        ValueError: if the given `duration` is not of a supported type.
    """
    if isinstance(duration, (int, float)):
        return Duration(seconds=duration)
    if isinstance(duration, timedelta):
        return Duration(seconds=duration.total_seconds())
    if not isinstance(duration, Duration):
        raise ValueError(f"unsupported duration type: {duration}")
    return duration
