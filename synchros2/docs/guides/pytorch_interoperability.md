# PyTorch Interoperability

In general, a ROS 2 node may trivially wrap a [PyTorch](https://pytorch.org/) model for inference (e.g. in a callback). One notable exception to this are nodes spinning on multi-threaded executors (as by default when using `synchros2`) and performing inference on GPUs. Thread-local contexts and CPU <> GPU synchronization make it so that, for best performance out of the box, models must always run on the same thread and never concurrently with others. ROS 2 and `synchros2` afford a couple idioms to deal with these constraints.

## Idioms

For illustrative purposes, code snippets below use sample models as listed in the appendix.

### Foreground inference only

Single-threaded execution precludes the aforementioned issues. This make it best suited for simple model wrappers:

```python
# sample_node.py

from typing import Any
from sensor_msgs.msg import Image

from synchros2.node import Node
from synchros2.executors import foreground
import synchros2.process as ros_process 

from rclpy.executors import SingleThreadedExecutor

from sample_models import MaskFormerROS

class MaskFormerROSNode(Node):

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__("sample_node", *args, **kwargs)
        self.segmentation = MaskFormerROS()
        self.pub = self.create_publisher(Image, "~/output/image", 1)
        self.sub = self.create_subscription(Image, "~/input/image", self.on_input_callback, 1)

    def on_input_callback(self, message: Image) -> None:
        self.pub.publish(self.segmentation.perform(message))

@ros_process.main(prebaked=False)
def main():
    with foreground(SingleThreadedExecutor()) as main.executor: 
        main.spin(MaskFormerROSNode) 

if __name__ == "__main__":
    main()
```

### Background inference, foreground threads

A single-threaded executor spinning in the background may be used for generic work dispatch. This can be handy in multi-threaded applications:

```python
# sample_node.py

from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Image
from synchros2.executors import background
from synchros2.futures import unwrap_future
import synchros2.process as ros_process 

from sample_models import MaskFormerROS

@ros_process.main(autospin=False)
def main():
    segmentation = MaskFormerROS()
    with background(SingleThreadedExecutor()) as background_executor:
        pub = main.node.create_publisher(Image, "~/output/image", 1)
        def on_input_callback(message: Image) -> None:
            pub.publish(unwrap_future(background_executor.create_task(segmentation.perform, message)))
        main.node.create_subscription(Image, "~/input/image", on_input_callback, 1)
        main.spin()  # until Ctrl + C 

if __name__ == "__main__":
    main()
```

### Background threads, foreground inference

Conversely, `synchros2` abstractions and patterns may be leveraged to bring back the simpler, linear code paths: 

```python
# sample_node.py

import contextlib

from sensor_msgs.msg import Image

import synchros2.process as ros_process
from synchros2.publisher import Publisher
from synchros2.subscription import Subscription

from sample_models import MaskFormerROS

@ros_process.main()
def main():
    segmentation = MaskFormerROS()
    publisher = Publisher(Image, "~/output/image")
    subscription = Subscription(Image, "~/input/image")
    with contextlib.closing(subscription.stream()) as stream:
        for image in stream:  # indefinitely until Ctrl + C
            publisher.publish(segmentation.perform(image))

if __name__ == "__main__":
    main()
```

### Callback groups with thread affinity

For the more complex (or reusable) setups, when there's less control over execution paths, `synchros2` executors support thread affinity settings for callback groups. That is, one or more callback groups may be configured to be served by specific thread pools of one or more workers (typically one when dealing with inference and GPU workloads in general):

```python
# sample_node.py

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image

import synchros2.process as ros_process

from sample_models import MaskFormerROS

@ros_process.main(autospin=False)
def main():
    thread_affine_callback_group = MutuallyExclusiveCallbackGroup()
    thread_pool = main.executor.add_static_thread_pool(1)
    main.executor.bind(thread_affine_callback_group, thread_pool)

    segmentation = MaskFormerROS()
    pub = main.node.create_publisher(Image, "~/output/image", 1)

    def on_input_callback(message: Image) -> None:
        pub.publish(segmentation.perform(message))

    main.node.create_subscription(
        Image, "~/input/image", on_input_callback, 1,
        callback_group=thread_affine_callback_group
    )

    main.spin()  # until Ctrl + C

if __name__ == "__main__":
    main()
```

## Appendix

Below, a sample [pretrained segmentation model](https://huggingface.co/docs/transformers/en/model_doc/mask2former) wrapped to interface with ROS messages:

```python
# sample_models.py

import cv2
import numpy as np
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from transformers import (
    AutoImageProcessor, 
    Mask2FormerForUniversalSegmentation,
)
import torch


def labels2rgb(labels: np.ndarray) -> np.ndarray:
    label_range = np.arange(np.min(labels), np.max(labels))
    lut = np.zeros((256, 1, 3), dtype=np.uint8)
    lut[:label_range[-1], 0, :] = np.uint8(
        256 * plt.cm.tab20(label_range / label_range[-1])[:,:-1]
    )
    return cv2.LUT(cv2.merge((labels, labels, labels)), lut)


class MaskFormerROS:

    bridge = CvBridge()

    def __init__(self) -> None:
        self.image_processor = AutoImageProcessor.from_pretrained("facebook/mask2former-swin-small-ade-semantic")
        self.model = Mask2FormerForUniversalSegmentation.from_pretrained("facebook/mask2former-swin-small-ade-semantic")

    def perform(self, message: Image) -> Image:
        image = self.bridge.imgmsg_to_cv2(message)
        
        inputs = self.image_processor(image, return_tensors="pt")

        with torch.no_grad():
            outputs = self.model(**inputs)

        class_queries_logits = outputs.class_queries_logits
        masks_queries_logits = outputs.masks_queries_logits

        pred_semantic_map = self.image_processor.post_process_semantic_segmentation(
            outputs, target_sizes=[image.shape]
        )[0].numpy().astype(np.uint8)

        return self.bridge.cv2_to_imgmsg(labels2rgb(pred_semantic_map), "rgb8")
```
