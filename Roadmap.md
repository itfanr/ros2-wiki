# ROS 2.0 Roadmap

For more information on the design of ROS 2.0 please see [design.ros2.org](http://design.ros2.org). 
The core code for ros2 is on the [ros2 github organization](https://github.com/ros2). The mailing list for discussing ROS 2.0 is [ros-sig-ng-ros](https://groups.google.com/d/forum/ros-sig-ng-ros).

## Alpha release

- Continue to iterate on design documents: http://design.ros2.org
- Support and evaluate more DDS/RTPS implementations:
 - Fast-RTPS: https://github.com/eProsima/Fast-RTPS
 - freertps: https://github.com/ros2/freertps
- Dynamic loading of nodes and plugins
 - a al `class_loader`/`pluginlib`
- Static remapping
 - a la ROS Names: http://wiki.ros.org/Names
- Component life-cycle:
 - Orchestration API
 - Introspection API
- Type masquerading
 - a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
- Launch system
- Dynamic remapping
 - Remapping and aliasing through a Service interface
- Graph API
 - a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
- Command line tools:
  - introspection, a la `rostopic`
  - console logging, a la `rosconsole`
  - data recording/playback, a la `rosbag`
- Multi-robot supporting features and demos
- Debian packaging
- ROS 1/ROS 2 Migration Guide
 - Tutorials and examples of migrating ROS 1 packages
- C client library
- Python client library

## Pre Alpha release

- Multi-platform support
 - Linux
 - OSX
 - Windows
 - RTOS or no OS
- Build system
- Message generation
- Abstract middleware interface
 - multiple DDS vendor support
  - OpenSplice
  - Connext
  - Connext with Dynamic Data support
- Executor model
- Publish/subscribe API
- Request/response (Services) API
 - Synchronous as well as asynchronous
- Dynamic parameter API

Demonstrations:
- ROS 1 <-> ROS 2 bridge
- Quality of Service in lossy environments
- Efficient intra-process communications
- Real-time controller
- Publish/Subscribe on an embedded system without an OS
