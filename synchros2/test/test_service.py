# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

import pytest
from rclpy.duration import Duration
from std_srvs.srv import Trigger

from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope
from synchros2.service import Serviced, ServiceError, ServiceTimeout


def succeeding_callback(request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
    response.success = True
    return response


def test_successful_synchronous_service_invocation(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    ros.node.create_service(Trigger, "trigger_something", succeeding_callback)
    trigger_something: Serviced[Trigger.Request, Trigger.Response] = Serviced(Trigger, "trigger_something")
    response = trigger_something(timeout_sec=5.0)
    assert response.success


def test_successful_asynchronous_service_invocation(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    ros.node.create_service(Trigger, "trigger_something", succeeding_callback)
    trigger_something: Serviced[Trigger.Request, Trigger.Response] = Serviced(Trigger, "trigger_something")
    service = trigger_something.asynchronously()
    assert wait_for_future(service, timeout_sec=5.0)
    response = service.result()
    assert response.success


def test_timed_out_synchronous_service_invocation(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    clock = ros.node.get_clock()

    def callback(request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        clock.sleep_until(clock.now() + Duration(seconds=5.0))
        response.success = True
        return response

    ros.node.create_service(Trigger, "trigger_something", callback)
    trigger_something: Serviced[Trigger.Request, Trigger.Response] = Serviced(Trigger, "trigger_something")
    with pytest.raises(ServiceTimeout):
        trigger_something(timeout_sec=0.01)


def failing_callback(request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
    response.success = False
    return response


def test_failing_synchronous_service_invocation(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    ros.node.create_service(Trigger, "trigger_something", failing_callback)
    trigger_something: Serviced[Trigger.Request, Trigger.Response] = Serviced(Trigger, "trigger_something")
    response = trigger_something(nothrow=True, timeout_sec=5.0)
    assert not response.success
    with pytest.raises(ServiceError) as exc:
        trigger_something(timeout_sec=5.0)
    response = exc.value.service.result()
    assert not response.success


def test_failing_asynchronous_service_invocation(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    ros.node.create_service(Trigger, "trigger_something", failing_callback)
    trigger_something: Serviced[Trigger.Request, Trigger.Response] = Serviced(Trigger, "trigger_something")
    service = trigger_something.asynchronously()
    assert wait_for_future(service, timeout_sec=5.0)
    response = service.result()
    assert not response.success
