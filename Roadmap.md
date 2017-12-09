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

- Revise IDL file format, consider desired features (grouping, comments, etc.), suitability of existing formats like `IDL 4.2`
- Support for non-ASCII strings in messages / services
- Progress on migration plan
- Reconsider 1-to-1 mapping of ROS nodes to DDS participants
- Reconsider mapping of namespaces to DDS partitions
- Python-based launch with stable API, introspectable, optional XML frontend
- Make `ament_cmake` available in ROS 1 and/or `catkin` available in ROS 2

#### Infrastructure and tools

- Building
  - Work towards a [[universal build tool|http://design.ros2.org/articles/build_tool.html]]
  - Support for `devel` and `pull request` jobs on the [[ROS 2 buildfarm|http://build.ros2.org]]
  - Support to generate "fat" packages / archives
  - Windows and Mac OS packages
- Release
  - Future releases of ROS 2 (as of summer 2018) will likely target the next Ubuntu LTS 18.04
  - Allow releasing packages into ROS 2 using ready-to-use tools
- Documentation
  - Platform for documentation (like wiki.ros.org), allow easy contributions as well as optionally facilitate a review process
  - Support for `doc` jobs on the [[ROS 2 buildfarm|http://build.ros2.org]]
  - Consider consolidating with design.ros.org
  - Provide three different kinds of content:
    - "demos" to show features and cover them with tests
    - "examples" to show a simple/minimalistic usage which might have multiple ways to do something
    - "tutorials" which contain more comments and anchors for the wiki (teaching one recommended way)

#### New features

- Expose matched publisher / subscriber count (rather than only based on the topic name)
- Actions in ROS 2
  - Add pre-emption for services
- Feature parity across languages
  - Parameters in C and Python
  - ROS Time concepts in Python
- rosbag native in ROS 2
- Minor logging improvements
  - Configuration specified in a file
  - C++ stream operators
- Command line parameters and parameters from a yaml file
- Provide standard way to create and use components
- Launch system using life-cycle and orchestration
- Additional Graph API features
  - a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
- Remapping
  - Static remapping, a la ROS Names: http://wiki.ros.org/Names
  - Dynamic remapping and aliasing through a Service interface
- Type masquerading
  - a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
- Expand on real-time safety
  - With FastRTPS
  - For services, clients, and parameters
  - Support deterministic ordering of executables in Executor (fair scheduling)
  - Expose more quality of service parameters related to real-time performance
  - Real-time-safe intra-process messaging
- Multi-robot supporting features and demos
- Implement C client library `rclc`
- Support more DDS / RTPS implementations:
  - Connext dynamic
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
