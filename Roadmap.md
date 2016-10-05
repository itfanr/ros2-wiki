# ROS 2.0 Roadmap

For more information on the design of ROS 2.0 please see [design.ros2.org](http://design.ros2.org).
The core code for ros2 is on the [ros2 github organization](https://github.com/ros2).
The mailing list for discussing ROS 2.0 is [ros-sig-ng-ros](https://groups.google.com/d/forum/ros-sig-ng-ros).

## Planned upcoming features

This is a list of the features targeted for development in the future.

*Subject to change.*

### Beta 1 (circa 2016-12)

The Beta 1 will target Ubuntu 16.04, Mac OS X 10.11 and Windows 10.
 * Composition
   * may use pluginlib and class_loader from ROS 1 for C++
 * QoS benchmarks
   * for example: unreliable comms, illustrated by wifi out-and-back
 * Design documents
 * Tutorials and examples
   * Migration guide
 * "rostopic list", "rostopic echo", and friends
 * Bridging services to/from ROS1 (in addition to topics)

### Nice to have by Beta 1:
 * Console logging
 * Orchestration
   * think “roslaunch + verification & dynamic behavior”


### Future
- Automatic API documentation generation
- Launch system
  - Use life-cycle and orchestration
- data logging, perhaps using rosbag (or a descendant of rosbag)
- Additional Graph API features
  - a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
- Type masquerading
  - a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
- Add actions and bridge them
- Static remapping
  - a la ROS Names: http://wiki.ros.org/Names
- Dynamic remapping
  - Remapping and aliasing through a Service interface
- Debian packaging, Windows packaging
- Finish intra-process
  - Make it thread-safe
- Expand on real-time safety
  - For services, clients, and parameters
  - Support deterministic ordering of executables in Executor (fair scheduling)
  - Expose more quality of service parameters related to real-time performance
- Multi-robot supporting features and demos
- Add pre-emption for services
- Real-time-safe intra-process messaging
- Implement rclc
- Gazebo support
- Use DDS C++ ISO PSM API
- Support more DDS / RTPS implementations:
  - freertps: https://github.com/ros2/freertps
  - Use RTI's micro implementation
- Run-time DDS implementation choice

### Reducing Technical Debt

- Fix flaky tests.
- Add missing Windows stuff.
- Synchronise / reconcile design docs with the implementation.
  - Pre-release retrospective review (APIs, docs, etc.)
- Address TODOs in code / docs

## Past releases

See [list of releases](Releases).