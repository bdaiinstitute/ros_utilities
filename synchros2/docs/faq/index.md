# FAQ

## Is `synchros2` an alternative to `rclpy`?

It is not an alternative, it is an extension. `synchros2` builds on top of `rclpy` to enable a synchronous programming model that is closer in spirit to `rospy`.
Still, `synchros2` is fully compatible with `rclpy` by construction.

## What are these deadlocks `synchros2` solves?

By default, `rclpy` will resort to a single threaded executor to dispatch callbacks. A blocking callback will prevent the rest from executing, which can easily
lead to deadlocks if the awaited event depends, directly or indirectly, on callback dispatch (e.g. message arrival). Multi-threaded executors mitigate these
effects but `rclpy` provided ones require careful configuration. `synchros2` default executor precludes this entirely.
