# ROS 2.0 Roadmap

For more information on the design of ROS 2.0 please see [design.ros2.org](http://design.ros2.org).
The core code for ros2 is on the [ros2 github organization](https://github.com/ros2).
The mailing list for discussing ROS 2.0 is [ros-sig-ng-ros](https://groups.google.com/d/forum/ros-sig-ng-ros).

## Upcoming targeted features

This is a list of the features targeted for development in the future.

- Continue to iterate on design documents: http://design.ros2.org
- Evaluate and support more DDS / RTPS implementations:
 - Fast-RTPS: https://github.com/eProsima/Fast-RTPS
 - freertps: https://github.com/ros2/freertps
- Component life-cycle:
 - Introspection and orchestration APIs
- C client library
- Python client library
- Static remapping
 - a la ROS Names: http://wiki.ros.org/Names
- Dynamic remapping
 - Remapping and aliasing through a Service interface
- Launch system
- Type masquerading
 - a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
- Graph API
 - a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
- Command line tools:
  - introspection, a la `rostopic`
  - console logging, a la `rosconsole`
  - data recording / playback, a la `rosbag`
- ROS 1 / ROS 2 Migration Guide
 - Tutorials and examples of migrating ROS 1 packages
- Debian packaging, Windows packaging
- Expand on real-time safety
  - For services, clients, and parameters
  - For intra-process communication
  - Support deterministic ordering of executables in Executor (fair scheduling)
  - Expose more quality of service parameters related to real-time performance
- Multi-robot supporting features and demos


## Pre Alpha release

Our first release contains the following features and supports the following demos.

Major Features:
- Multi-platform support
 - Linux
 - OS X
 - Windows
 - RTOS or no OS
- Build system
- Message generation
- Abstract middleware interface
 - Multiple DDS vendor support
  - OpenSplice
  - Connext
  - Connext with Dynamic Data support
- Executor model
- Publish / subscribe API
- Request / response (Services) API
 - Synchronous as well as asynchronous
- Dynamic parameter API

Demonstrations:
- ROS 1 <-> ROS 2 bridge
- Quality of Service in lossy environments
- Efficient intra-process communications
- Real-time controller
- Publish/Subscribe on an embedded system without an OS
