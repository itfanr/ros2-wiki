# ROS 2 Roadmap

For more information on the design of ROS 2 please see [design.ros2.org](http://design.ros2.org).
The core code for ROS 2 is on the [ros2 github organization](https://github.com/ros2).
The Discourse forum/mailing list for discussing ROS 2 is [ng-ros](https://discourse.ros.org/c/ng-ros).

## Planned upcoming releases

This is a list of the features targeted for development in the future.

*Subject to change.*

### Beta 3 (September 13th, 2017)
Additions and changes to the following feature list will occur as time progresses.

- Parameters in C
- Feature parity in Python with execution model, wait-for-service, ros2 param
- Time: simtime, C++, Python
- Progress on migration plan, consider use of catkin API shim
- IDL file format, consider desired features, suitability of existing formats
- ros_control
- buildfarm: upstream ROS 2 spec. changes, one machine deployment, CI/devel/PR jobs

- Exposure of DDS implementation-specific symbols to users
- Expose logging macros in Python
- rviz native in ROS2
- Support more DDS / RTPS implementations:
  - OpenSplice

### Version 1.0 (Dec. 13th, 2017)

The feature list for version 1.0 will be filled out as Beta 3 progresses.

### Future

#### Fix known limitations

- FastRTPS performance with larger data like the image demo

#### New features (in no specific order)
- rosbag native in ROS2
- Logging configuration, C++ streams
- Command line static remapping
- Command line parameters and parameters from a yaml file
- Actions in ROS2
- Support services with Connext in C / Python
- Support for non-ASCII strings in messages / services
- Support DDS-Security specification
- Launch system using life-cycle and orchestration
- Add pre-emption for services
- Additional Graph API features
  - a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
- Static remapping
  - a la ROS Names: http://wiki.ros.org/Names
- Dynamic remapping
  - Remapping and aliasing through a Service interface
- Type masquerading
  - a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
- Finish intra-process making it thread-safe
- Expand on real-time safety
  - With FastRTPS
  - For services, clients, and parameters
  - Support deterministic ordering of executables in Executor (fair scheduling)
  - Expose more quality of service parameters related to real-time performance
  - Real-time-safe intra-process messaging
- Multi-robot supporting features and demos
- Implement rclc
- Support more DDS / RTPS implementations:
  - Connext dynamic
  - freertps: https://github.com/ros2/freertps
  - RTI's micro implementation
- Gazebo support
- robot_pose_ekf or robot_localization
- move_base

#### Infrastructure

- New documentation platform deployed
- Windows and OSX packages

### Reducing Technical Debt

- Fix flaky tests.
- Synchronize / reconcile design docs with the implementation.
  - Pre-release retrospective review (APIs, docs, etc.)
- Address / classify pending tickets
- Address TODOs in code / docs

## Past releases

See [list of releases](Releases).