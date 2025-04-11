# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
import argparse
import threading
import unittest.mock as mock

import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.task import Future
from std_srvs.srv import Trigger

import synchros2.process as process
from synchros2.futures import wait_for_future
from synchros2.static_transform_broadcaster import StaticTransformBroadcaster


def test_process_wrapping() -> None:
    """Asserts that the process bound node is made available."""

    @process.main()
    def main() -> int:
        def dummy_server_callback(_: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            response.success = True
            return response

        assert main.node is not None
        main.node.create_service(Trigger, "/dummy/trigger", dummy_server_callback)

        client = main.node.create_client(Trigger, "/dummy/trigger")
        assert client.wait_for_service(timeout_sec=10)

        response = client.call(Trigger.Request())
        assert response.success
        return 0

    assert main() == 0


def test_process_exception_propagates() -> None:
    """Asserts that any exception raised within the main thread propagates."""

    barrier = threading.Barrier(2)

    def wait_forever() -> None:
        barrier.wait()
        wait_for_future(Future())

    @process.main()
    def main() -> int:
        assert main.executor is not None
        main.executor.create_task(wait_forever)
        barrier.wait()
        raise RuntimeError("probe")

    with pytest.raises(RuntimeError, match="probe"):
        main()


def test_process_using_tf() -> None:
    """Asserts that the process bound tf listener is made available."""

    @process.main(uses_tf=True)
    def main() -> int:
        assert main.node is not None
        stamp = main.node.get_clock().now().to_msg()

        expected_transform = TransformStamped()
        expected_transform.header.stamp = stamp
        expected_transform.header.frame_id = "world"
        expected_transform.child_frame_id = "robot"
        expected_transform.transform.rotation.w = 1.0
        tf_broadcaster = StaticTransformBroadcaster(main.node)
        tf_broadcaster.sendTransform(expected_transform)

        assert main.tf_listener is not None
        transform = main.tf_listener.lookup_a_tform_b("world", "robot", stamp, timeout_sec=5.0, wait_for_frames=True)
        assert transform.header.stamp == expected_transform.header.stamp
        assert transform.transform.rotation.w == expected_transform.transform.rotation.w
        return 0

    assert main() == 0


def test_command_wrapping() -> None:
    """Asserts that the process bound node is made available."""

    def cli() -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser("test_command")
        parser.add_argument("robot")
        return parser

    @process.main(cli())
    def main(args: argparse.Namespace) -> int:
        assert main.node is not None
        assert main.node.get_fully_qualified_name() == "/test_command"
        assert args.robot == "spot"
        return 0

    assert main(["test_command", "spot"]) == 0


def test_cli_configuration() -> None:  # type: ignore
    """Asserts that CLI can affect process configuration."""

    def cli() -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser("test_command")
        parser.add_argument("robot")
        parser.add_argument("-q", "--quiet", action="store_true")
        parser.set_defaults(process_args=lambda args: dict(forward_logging=not args.quiet))
        return parser

    @process.main(cli(), namespace="/{robot}")
    def main(args: argparse.Namespace) -> int:
        assert main.node is not None
        assert main.node.get_fully_qualified_name() == "/spot/test_command"
        assert args.robot == "spot"
        return 0

    with mock.patch("synchros2.scope.logs_to_ros") as logs_to_ros:
        assert main(["test_command", "spot", "--quiet"]) == 0
    assert not logs_to_ros.called
