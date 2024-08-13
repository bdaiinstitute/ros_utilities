#  Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
import inspect
from typing import Optional, Tuple

from rclpy.client import Client
from std_srvs.srv import Empty, SetBool, Trigger

from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.service_handle import ServiceHandle
from bdai_ros2_wrappers.type_hints import SrvTypeRequest, SrvTypeResponse


def do_request(
    client: Client,
    request: SrvTypeRequest,
    label: Optional[str] = None,
) -> Tuple[bool, Optional[SrvTypeResponse]]:
    if label is None:
        label = inspect.stack()[1].function
    assert client.wait_for_service(timeout_sec=1.0)

    failure = False

    def failure_callback() -> None:
        nonlocal failure
        failure = True

    handle = ServiceHandle(label)
    handle.set_on_failure_callback(failure_callback)

    service_future = client.call_async(request)
    handle.set_send_service_future(service_future)

    assert handle.wait(timeout_sec=1.0)
    return not failure, handle.result


def test_successful_trigger(ros: ROSAwareScope) -> None:
    """Request a successful trigger. Should return a success condition with the message foo."""

    def callback(req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        result = Trigger.Response()
        result.success = True
        result.message = "foo"
        return result

    assert ros.node is not None
    ros.node.create_service(Trigger, "trigger_service", callback)
    client = ros.node.create_client(Trigger, "trigger_service")
    ok, result = do_request(client, Trigger.Request())

    assert ok
    assert result is not None
    assert result.success
    assert result.message == "foo"


def test_failed_trigger(ros: ROSAwareScope) -> None:
    """Request a failed trigger. Should return a success condition with the message bar."""

    def callback(req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        result = Trigger.Response()
        result.success = False
        result.message = "bar"
        return result

    assert ros.node is not None
    ros.node.create_service(Trigger, "trigger_service", callback)
    client = ros.node.create_client(Trigger, "trigger_service")

    ok, result = do_request(client, Trigger.Request())

    assert not ok
    assert result is not None
    assert not result.success
    assert result.message == "bar"


def setbool_service_callback(req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
    result = SetBool.Response()
    if req.data:
        result.success = True
        result.message = "foo"
    else:
        result.success = False
        result.message = "bar"
    return result


def test_successful_set_bool(ros: ROSAwareScope) -> None:
    """Request data passes a true value. Should return a success condition with the message foo."""
    assert ros.node is not None
    ros.node.create_service(SetBool, "setbool_service", setbool_service_callback)
    client = ros.node.create_client(SetBool, "setbool_service")
    ok, result = do_request(client, SetBool.Request(data=True))

    assert ok
    assert result is not None
    assert result.success
    assert result.message == "foo"


def test_failed_set_bool(ros: ROSAwareScope) -> None:
    """Request data passes a false value. Should return a failure condition with the message bar."""
    assert ros.node is not None
    ros.node.create_service(SetBool, "setbool_service", setbool_service_callback)
    client = ros.node.create_client(SetBool, "setbool_service")
    ok, result = do_request(client, SetBool.Request(data=False))

    assert not ok
    assert result is not None
    assert not result.success
    assert result.message == "bar"


def test_warning(ros: ROSAwareScope) -> None:
    """Tests that the result callback should give a warning and have no fields."""

    def callback(req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        return Empty.Response()

    assert ros.node is not None
    ros.node.create_service(Empty, "empty_service", callback)
    client = ros.node.create_client(Empty, "empty_service")
    ok, result = do_request(client, Empty.Request())

    assert ok
    assert result is not None
    assert isinstance(result, Empty.Response)
    assert not hasattr(result, "success")
