# ROS 2.0 Roadmap

This document is designed to capture the plans for upcoming ROS 2.0 development milestones. 

For more information on the design of ROS 2.0 please see [design.ros2.org](http://design.ros2.org). 
The core code for ros2 is on the [ros2 github organization](https://github.com/ros2). The mailing list for discussing ROS 2.0 is [ros-sig-ng-ros](https://groups.google.com/d/forum/ros-sig-ng-ros)

## Preview +1 release

- finalize more design documents (design.ros2.org)
- add support for FastRTPS
- add support for freertps
- dynamic loading objects(class_loader/pluginlib)
- static remapping
- component model
 - orchestration API
 - introspection API
- type masquarading
- launch system
- dynamic remapping
- graph API
- develop command line tools
  - introspection e.g. rostopic
  - logging e.g. rosconsole
  - data recording/playback
- multi-robot 
- build debian packages
- write up full migration guide
- develop examples of porting from ROS1 to ROS2
- C client library
- Python client library

## Preview release

- cross platform support
 - Linux, OSX, Windows, (bare metal/RTOS)
- build system
- message generation
- abstract dds interface
 - multiple DDS vendor support
  - OpenSplice
  - Connext
- executor model
- publish subscribe support
- request/response (services) support
 - synchronous + asynchronous
- dynamic parameter support support

Demonstrations
- ros1_bridge
- quality of service
- efficient intraprocess communications
- realtime controller
- baremetal embedded pub/sub
