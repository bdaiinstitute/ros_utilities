# `synchros2`

`synchros2` is a collection of utilities and wrappers built on top of [`rclpy`](https://github.com/ros2/rclpy). These utilities simplify ROS 2 usage by enabling standard, idiomatic, synchronous Python programming. To that end, `synchros2` relies on heavy yet implicit concurrency and thus there is overhead in its simplicity.relying on implicit concurrency for simplicity.

## Features

- Process-wide ROS 2 nodes
- RPC-like action and service clients
- Stateful stream-like topic subscriptions
