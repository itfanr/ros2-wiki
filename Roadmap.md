# ROS 2.0 Roadmap

For more information on the design of ROS 2.0 please see [design.ros2.org](http://design.ros2.org).
The core code for ros2 is on the [ros2 github organization](https://github.com/ros2).
The mailing list for discussing ROS 2.0 is [ros-sig-ng-ros](https://groups.google.com/d/forum/ros-sig-ng-ros).

## Planned upcoming features

This is a list of the features targeted for development in the future.

*Subject to change.*

### Alpha 7 (circa 2016-07)

See https://github.com/ros2/ros2/issues/225

### Future

- rcl
  - support for services
- rclcpp
  - refactor to use rcl
- Python client library
- Component life-cycle:
  - Introspection and orchestration APIs
  - Use [[class_loader|https://github.com/ros/class_loader/tree/ros2]] / [[pluginlib|https://github.com/ros/pluginlib/tree/ros2]]
- Launch system
  - Use life-cycle and orchestration
- Real-time-safe intra-process messaging
- FreeRTPS improvements:
  - RMW support
- Graph API
  - a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
- Command line tools:
  - `rostopic` and friends
- ROS 1 / ROS 2 Migration Guide
  - Tutorials and examples of migrating ROS 1 packages
- Type masquerading
  - a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
- console logging, a la `rosconsole`
- Add actions
- Implement rclc
- Gazebo support
- Continue to iterate on design documents: http://design.ros2.org
- Evaluate and support more DDS / RTPS implementations:
  - Fast-RTPS: https://github.com/eProsima/Fast-RTPS
  - freertps: https://github.com/ros2/freertps
- Static remapping
  - a la ROS Names: http://wiki.ros.org/Names
- Dynamic remapping
  - Remapping and aliasing through a Service interface
- Debian packaging, Windows packaging
- Finish intra-process
  - make it real-time- / thread-safe
- Expand on real-time safety
  - For services, clients, and parameters
  - Support deterministic ordering of executables in Executor (fair scheduling)
  - Expose more quality of service parameters related to real-time performance
- Multi-robot supporting features and demos
- Use DDS C++ ISO PSM API
- Use RTI's micro implementation
- Make rclcpp a library
  - Make rclcpp more modular / capable of being composed
- Add pre-emption for services
- Implement actions
- Build packages in parallel
- Run-time DDS implementation choice

### Reducing Technical Debt

- Fix failing and flaky tests.
- Add missing Windows stuff.
- Synchronise / reconcile design docs with the implementation.
  - Pre-release retrospective review (APIs, docs, etc.)
- Address TODOs in code / docs
- Address important issues (e.g., services are broken on Windows)

## Past releases

See [list of releases](Releases).