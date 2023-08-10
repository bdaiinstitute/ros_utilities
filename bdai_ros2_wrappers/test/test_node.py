# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import multiprocessing
import time
import unittest
from threading import Thread
from typing import Optional

import rclpy
from rclpy import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from bdai_ros2_wrappers.node import NodeWrapper

# Used for testing that result from future
Test_Result = "success"


def thread_function(future: rclpy.Future, thread_sleep: int) -> None:
    """
    Helper thread process that takes a future to be
    set later for the node testing
    """
    # spin for a second then set the future
    time.sleep(thread_sleep)
    # set future result
    future.set_result(Test_Result)


def future_test_helper(node: NodeWrapper, timeout_sec: float = 2, thread_sleep: float = 1) -> Optional[str]:
    """
    Helper function to run a node wrapper and have it spin on a future result
    """
    # make a thread to test receiving a future
    future = rclpy.Future()
    test_thread = Thread(
        target=thread_function,
        args=(
            future,
            thread_sleep,
        ),
    )
    test_thread.start()

    # set spinning node to wait until complete
    if not node.spin_until_future_complete(future, timeout_sec=timeout_sec):
        return None

    return future.result()


class NodeWrapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context: Optional[Context] = Context()
        rclpy.init(context=self.context)

    def tearDown(self) -> None:
        rclpy.shutdown(context=self.context)
        self.context = None

    def test_node_configuration(self) -> None:
        """
        Tests the various node configuration and makes sure that they match what is set.
        """
        # Test single threaded node
        test_singlethread_node = NodeWrapper(node_name="Test_SingleThread", context=self.context)
        self.assertTrue(type(test_singlethread_node._executor) is SingleThreadedExecutor, "Single Thread Test")
        # Test multithreaded node
        test_multithread_node = NodeWrapper(node_name="Test_MultiThread", context=self.context, num_executor_threads=2)
        self.assertTrue(type(test_multithread_node._executor) is MultiThreadedExecutor, "MultiThread Thread Test")
        self.assertEqual(test_multithread_node._executor._executor._max_workers, 2, "Number of threads")
        # Test full cpu multithreaded node
        test_multithread_full_node = NodeWrapper(
            node_name="Test_MultiThread_Full", context=self.context, num_executor_threads=-1
        )
        self.assertTrue(type(test_multithread_full_node._executor) is MultiThreadedExecutor, "MultiThread Thread Test")
        self.assertEqual(
            test_multithread_full_node._executor._executor._max_workers,
            multiprocessing.cpu_count(),
            "Number of threads",
        )
        # Test Spin
        test_spin_node = NodeWrapper(node_name="Test_Spin", context=self.context, spin_thread=True)
        self.assertTrue(test_spin_node._thread is not None, "Thread Setting")
        # Check other thread that should not have a thread
        self.assertTrue(test_multithread_node._thread is None, "Thread None Setting")

        # shutdown
        test_singlethread_node.shutdown()
        test_multithread_node.shutdown()
        test_multithread_full_node.shutdown()
        test_spin_node.shutdown()

    def test_spin_until_future_complete(self) -> None:
        """
        Tests the spin until future complete on both a pre-spinning node and non-spinning node which have different
        behavior
        """
        # create a spinning and non-spinning node to test both code pathways
        test_spinning_node = NodeWrapper(node_name="Spinning_Node", context=self.context, spin_thread=True)
        test_non_spinning = NodeWrapper(node_name="Nonspinning_Node", context=self.context)

        # set spinning node to wait until complete
        res = future_test_helper(test_spinning_node)
        self.assertEqual(res, Test_Result)

        # set non-spinning node to wait and restart test_thread
        res = future_test_helper(test_non_spinning)
        self.assertEqual(res, Test_Result)

        # test timeouts by setting the future setting thread to a longer period than the timeout
        self.assertEqual(future_test_helper(test_spinning_node, 0.5, 2), None)
        # test timout on non-spinning node (rclpy does not throw exceptions on timeouts so check for none future result)
        self.assertEqual(future_test_helper(test_non_spinning, 0.5, 2), None)

        # shutdown
        test_non_spinning.shutdown()
        test_spinning_node.shutdown()


if __name__ == "__main__":
    unittest.main()
