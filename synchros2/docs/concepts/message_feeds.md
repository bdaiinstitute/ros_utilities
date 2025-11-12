# Message Feeds

Message feeds generalize standard ROS 2 message filters, allowing latest message retrieval, message history, callback registration, and streaming. 

Any message filter can become a feed, allowing:

- latest message retrieval
- message history retrieval
- message callback registration (and de-registration)
- message stream iteration as they arrive, one-by-one or in batches
- incoming message waits -- any message or those matching a predicate

Like message filters, most message feeds can be chained. This is true for all but those that externally source messages, ROS 2 topic subscriptions being the prime example. These are sources only. Other message feeds built into `synchros2` offer a vehicle for generic map-filter-reduce patterns, time synchronization across multiple message feeds, and synchronized `tf` lookups.

**Note:** While any message filter can become a feed, standard ROS 2 message filters are usually not thread-safe. See [`synchros2.filters`](https://github.com/bdaiinstitute/synchros2/tree/main/synchros2/synchros2/filters.py) for thread-safe (re)implementations.

## Looping over topic messages

```python
from contextlib import closing
from std_msgs.msg import String
from synchros2.subscription import Subscription
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    topic_data = Subscription(String, "topic")
    with closing(topic_data.stream()) as stream:
        for message in stream:
            print(message.data)

if __name__ == "__main__":
    main()
```

Note that the topic message stream is managed and closed explicitly by the `contextlib.closing` context manager. This is important to stop message buffering as soon as it is no longer necessary.

## Waiting for the next topic message

```python
from std_msgs.msg import String
from synchros2.subscription import Subscription
from synchros2.futures import unwrap_future
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    topic_data = Subscription(String, "topic")
    while main.context.ok():
        message = unwrap_future(topic_data.update, timeout_sec=5.0)
        print(message.data)

if __name__ == "__main__":
    main()
```

Note that the future update does not become available in time, future unwrapping will raise.

## Waiting for a specific topic message

```python
from std_msgs.msg import Int32
from std_msgs.msg import String
from synchros2.subscription import Subscription
from synchros2.futures import unwrap_future
import synchros2.process as ros_process

def to_int32(message: String) -> Int32:
    return Int32(data=int(message.data.rpartition(" ")[-1]))

@ros_process.main()
def main() -> None:
    topic_data = Subscription(String, "topic")
    while main.context.ok():
        message = unwrap_future(topic_data.matching_update(
            lambda message: to_int32(message).data % 5 == 0
        ), timeout_sec=5.0)
        print(message.data)
if __name__ == "__main__":
    main()
```

## Setting up a message callback

```python
from std_msgs.msg import String
from synchros2.subscription import Subscription
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    topic_data = Subscription(String, "topic")
    topic_data.recall(lambda message: print(message.data))
    main.wait_for_shutdown()
if __name__ == "__main__":
    main()
```

## Synchronizing topic messages

```python
from contextlib import closing
from std_msgs.msg import String
from synchros2.feeds import SynchronizedMessageFeed
from synchros2.futures import unwrap_future
from synchros2.subscription import Subscription
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    topics_data = SynchronizedMessageFeed(
        Subscription(String, "topic0"),
        Subscription(String, "topic1"),
        allow_headerless=True
    )
    with closing(topics_data.stream()) as stream:
        for a, b in stream:
            print("a:", a.data, "matches b:", b.data)

if __name__ == "__main__":
    main()
```

## Adapting topic messages

```python
from contextlib import closing
from typing import Optional
from std_msgs.msg import String
from std_msgs.msg import Int32
from synchros2.feeds import AdaptedMessageFeed
from synchros2.subscription import Subscription
import synchros2.process as ros_process

def to_int32(message: String) -> Int32:
    return Int32(data=int(message.data.rpartition(" ")[-1]))

def keep_even(message: Int32) -> Optional[Int32]:
    return message if message.data % 2 == 0 else None

@ros_process.main()
def main() -> None:
    topic_data = AdaptedMessageFeed(
        Subscription(String, "topic"),
        lambda msg: keep_even(to_int32(msg))
    )
    with closing(topic_data.stream()) as stream:
        for message in stream:
            print(message.data)

if __name__ == "__main__":
    main()
```

Note that the adapter logic not only transforms the message type but also filters them. Returning `None` stops message propagation down the chain of message feeds.

## Fetch the last 10 topic messages

```python
from contextlib import closing
from std_msgs.msg import String
from synchros2.subscription import Subscription
from synchros2.time import Duration
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    clock = main.node.get_clock()
    with closing(Subscription(String, "topic", history_length=10)) as topic_data:
        print("Waiting for 5 seconds...")
        clock.sleep_until(clock.now() + Duration(seconds=5.0))
    for message in topic_data.history:
        print(message.data)

if __name__ == "__main__":
    main()
```
