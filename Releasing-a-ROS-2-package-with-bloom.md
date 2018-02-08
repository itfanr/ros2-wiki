## Required tool versions

For ROS 2 *ardent*:

* `bloom` >= 0.6.2
* `catkin_pkg` >= 0.4.0

## Setting up the environment

ROS 2 uses a forked rosdistro index located at https://github.com/ros2/rosdistro.
You can configure bloom to use it by setting the `ROSDISTRO_INDEX_URL` environment variable.

```
export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'
```

## Running bloom

No additional changes should be required to run bloom.
The release process for early ROS 2 releases primarily relied on the `git-bloom-release` subcommand rather than the full `bloom-release` workflow so there may be issues with it.
