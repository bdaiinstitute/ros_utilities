# Copyright (c) 2025 Boston Dynamics AI Institute LLC.  All rights reserved.

import threading
import time

import pytest
from rclpy.clock import Clock
from rclpy.duration import Duration

from synchros2.clock import wait_for
from synchros2.process import ROSAwareScope


@pytest.fixture
def ros_clock(ros: ROSAwareScope) -> Clock:
    clock = ros.node.get_clock()
    clock._set_ros_time_is_active(True)
    stop_event = threading.Event()

    def loop() -> None:
        while not stop_event.is_set():
            t = clock.now() + Duration(seconds=0.1)
            clock.set_ros_time_override(t)
            time.sleep(0.1)

    thread = threading.Thread(target=loop)
    thread.start()
    try:
        yield clock
    finally:
        stop_event.set()
        thread.join()


def test_wait_for_impossible_event_wrt_system_clock() -> None:
    impossible_event = threading.Event()
    assert not wait_for(impossible_event, timeout_sec=0.1)
    assert not impossible_event.is_set()


def test_wait_for_past_event_wrt_system_clock() -> None:
    past_event = threading.Event()
    past_event.set()
    assert wait_for(past_event, timeout_sec=0.5)
    assert past_event.is_set()


def test_wait_for_late_event_wrt_system_clock(ros: ROSAwareScope) -> None:
    late_event = threading.Event()
    ros.executor.create_task(late_event.set)
    assert wait_for(late_event, timeout_sec=0.5)
    assert late_event.is_set()


def test_wait_for_impossible_event_wrt_ros_clock(ros_clock: Clock) -> None:
    impossible_event = threading.Event()
    assert not wait_for(impossible_event, clock=ros_clock, timeout_sec=0.5)
    assert not impossible_event.is_set()


def test_wait_for_past_event_wrt_ros_clock(ros_clock: Clock) -> None:
    past_event = threading.Event()
    past_event.set()
    assert wait_for(past_event, clock=ros_clock, timeout_sec=0.5)
    assert past_event.is_set()


def test_wait_for_late_event_wrt_ros_clock(ros: ROSAwareScope, ros_clock: Clock) -> None:
    late_event = threading.Event()
    ros.executor.create_task(late_event.set)
    assert wait_for(late_event, clock=ros_clock, timeout_sec=0.5)
    assert late_event.is_set()
