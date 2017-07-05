# ROS 2 Roadmap

For more information on the design of ROS 2 please see [design.ros2.org](http://design.ros2.org).
The core code for ROS 2 is on the [ros2 github organization](https://github.com/ros2).
The Discourse forum/mailing list for discussing ROS 2 is [ng-ros](https://discourse.ros.org/c/ng-ros).

## Planned upcoming releases

This is a list of the features targeted for development in the future.

*Subject to change.*

### Beta 2 (July 7th, 2017)
- ROS 2 native support for:
  - Cartographer
  - AMCL
  - astra driver
  - map_server
  - joystick_driver
  - Kobuki driver
- Support via the ROS <--> ROS2 bridge:
  - rosbag
  - rviz
- Released packages, at least Ubuntu
- Namespaced topics and services
- rostopic command line tool (not all verbs)
- Logging API (rosconsole)
- Run-time DDS implementation choice, avoid having to build N variations of all libraries / executables

### Beta 3 (September 13th, 2017)
Additions and changes to the following feature list will occur as Beta 2 progresses.

- Windows and OSX packages
- Provide catkin API through an ament package
- rclcpp::Time
- rviz and rosbag native in ROS2
- Actions in ROS2
- rosparam
- parameters in C
- ros_control, perception, depth camera 
- move_base
- robot_pose_ekf or robot_localization
- Command line static remapping
- Command line parameters and parameters from a yaml file
- New documentation platform deployed

### Version 1.0 (Dec. 13th, 2017)

The feature list for version 1.0 will be filled out as Beta 2 and Beta 3 progress.

### Future

#### Fix known limitations

- FastRTPS performance with larger data like the image demo

#### New features (in no specific order)
- Complete Python API with execution model
- Support services with Connext in C / Python
- Exposure of DDS implementation-specific symbols to users
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
  - OpenSplice
  - freertps: https://github.com/ros2/freertps
  - RTI's micro implementation
- Gazebo support

#### Infrastructure

- Automatic API documentation generation
- Debian packaging, Windows packaging

### Reducing Technical Debt

- Fix flaky tests.
- Synchronize / reconcile design docs with the implementation.
  - Pre-release retrospective review (APIs, docs, etc.)
- Address / classify pending tickets
- Address TODOs in code / docs

## Past releases

See [list of releases](Releases).