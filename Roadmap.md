# ROS 2 Roadmap

For more information on the design of ROS 2 please see [design.ros2.org](http://design.ros2.org).
The core code for ROS 2 is on the [ros2 github organization](https://github.com/ros2).
The mailing list for discussing ROS 2 is [ros-sig-ng-ros](https://groups.google.com/d/forum/ros-sig-ng-ros).

## Planned upcoming features

This is a list of the features targeted for development in the future.

*Subject to change.*

### Beta 2 (circa June 2017)
- Run-time DDS implementation choice, avoid having to build N variations of all libraries / executables
- Support for namespaced topics/services
- Complete Python API with execution model
- Support services with Connext in C / Python
- Support for parameters in C / Python
- Exposure of DDS implementation-specific symbols to users
- Support for non-ASCII strings in messages / services
- Support DDS-Security specification

### Future

#### Fix known limitations

- FastRTPS performance with larger data like the image demo


#### New features (in no specific order)

- Console logging API
- Command line tools
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
- Add actions and support for them in the ros1_bridge
- Data logging, perhaps using rosbag (or a descendant of rosbag)
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