# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

from std_srvs.srv import Trigger

import bdai_ros2_wrappers.process as process


def test_process_wrapping() -> None:
    """Asserts that the process bound node is made available."""

    @process.main(name="test_process")
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
