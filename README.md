# ros_utilities
Wrappers and other utilities for ROS2

## Usage

See the [`ros_utilities` wiki](https://github.com/bdaiinstitute/ros_utilities/wiki) on how to make the best out of these utilities.

## Contribution
To contribute, install `pre-commit` via pip, run `pre-commit install` and then run `pre-commit run --all-files` to
verify that your code will pass inspection.
```bash
git clone https://github.com/bdaiinstitute/ros_utilities.git
cd ros_utilities
pip3 install pre-commit
pre-commit install
pre-commit run --all-files
```

Now whenever you commit code to this repository, it will be checked against our `pre-commit` hooks. You can also run
`git commit --no-verify` if you wish you commit without checking against the hooks.
