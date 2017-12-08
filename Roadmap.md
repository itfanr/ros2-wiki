# ROS 2 Roadmap

For more information on the design of ROS 2 please see [design.ros2.org](http://design.ros2.org).
The core code for ROS 2 is on the [ros2 github organization](https://github.com/ros2).
The Discourse forum/mailing list for discussing ROS 2 is [ng-ros](https://discourse.ros.org/c/ng-ros).

## Planned upcoming releases

This is a list of the features targeted for development in the future.

*Subject to change.*

### Next release (Summer 2018)

The feature list for this release will be filled out after the *Ardent* release.

- ...<to be filled>...

### Future (in no specific order)

#### Design / Concept

- Revise IDL file format, consider desired features, suitability of existing formats
- Reconsider 1-to-1 mapping of ROS nodes to DDS participants
- Support for non-ASCII strings in messages / services
- Progress on migration plan
- Reconsider mapping of namespaces to DDS partitions

### Infrastructure

- Support for `devel`, `pull request`, and `doc` jobs on the [[ROS 2 buildfarm|http://build.ros2.org]]
- Platform for documentation, allow easy contributions as well as optionally facilitate a review process
- Buildfarm: CI/devel/PR jobs, fat packages / archives, one-machine deployment for automated testing
- Windows and MacOS packages

#### New features

- Parameters in C and Python
- ROS Time concepts in Python
- Logging configuration
- Expose matched publisher / subscriber count
- rosbag native in ROS2
- Logging C++ streams
- Command line parameters and parameters from a yaml file
- Actions in ROS2
- Add pre-emption for services
- Provide standard way to create and use components
- Launch system using life-cycle and orchestration
- Additional Graph API features
  - a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
- Static remapping
  - a la ROS Names: http://wiki.ros.org/Names
- Dynamic remapping
  - Remapping and aliasing through a Service interface
- Type masquerading
  - a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
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
- security improvements:
  - more granularity in security configuration (allow authentication only, authentication and encryption, etc)
  - extend security to services, not only topics
  - integrate DDS-Security logging plugin (unified way to aggregate security events and report them to the users through a ROS interface)
  - key storage security (right now, keys are just stored in the filesystem)
  - more user friendly interface (make it easier to specify security config). Maybe a Qt GUI? This GUI could also assist in distributing keys somehow.
  - A way to say "please secure this running system" with some UI that would auto-generate keys and policies for everything that is currently running.
  - Map security features onto any other implementation that supports them.
  - If there are hardware-specific features for securing keys or accelerating encryption/signing messages, that could be interesting to add to DDS/RTPS implementations that don't use it already.

#### Reducing Technical Debt

- Fix flaky tests.
- API review
- Synchronize / reconcile design docs with the implementation.
  - Pre-release retrospective review (APIs, docs, etc.)
- Address / classify pending tickets
- Address TODOs in code / docs

## Past releases

See [list of releases](Releases).
