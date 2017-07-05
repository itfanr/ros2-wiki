### Beta 2 (codename r2b2; July 2017)

Welcome to the latest release of ROS 2 software! We hope that you try it out and provide feedback.

### Supported Platforms

We support ROS 2 Beta 2 on three platforms: Ubuntu 16.04 (Xenial), Mac OS X 10.12 (Sierra), and Windows 10.
We provide both binary packages and instructions for how to compile from source for all 3 platforms (see [install instructions](Installation) as well as [documentation](http://docs.ros2.org/beta2/)).

### Features

Improvements since the Beta 1 release:
* DDS_Security support (aka SROS2, see [sros2](https://github.com/ros2/sros2))
* Debian packages for Ubuntu Xenial (see [[Debian install instructions|Linux-Install-Debians]]).
* Typesupport has been redesigned so that you only build a single executable and can choose one of the available RMW implementations by setting an environment variable (see [[documentation|Working-with-multiple-RMW-implementations]]).
* Namespace support for nodes and topics (see [design article](http://design.ros2.org/articles/topic_and_service_names.html), see known issues below).
* A set of command line tools using the extensible `ros2` invocation (see [ros2cli](https://github.com/ros2/ros2cli) repository).
* A set of macros for logging messages in C / C++ (see API docs of [rcutils](http://docs.ros2.org/beta2/api/rcutils/index.html)).

New demo application:
* [Turtlebot 2 demos](https://github.com/ros2/turtlebot2_demo) using the following repositories that have been (partially) converted to ROS 2:
  * [ros_astra_camera](https://github.com/ros2/ros_astra_camera.git)
  * [depthimage_to_laserscan](https://github.com/ros2/depthimage_to_laserscan.git)
  * [pcl_conversions](https://github.com/ros2/pcl_conversions.git)
  * [cartographer](https://github.com/ros2/cartographer.git)
  * [cartographer_ros](https://github.com/ros2/cartographer_ros.git)
  * [ceres-solver](https://github.com/ros2/ceres-solver.git)
  * [navigation](https://github.com/ros2/navigation.git)
  * [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard.git)
  * [joystick_drivers](https://github.com/ros2/joystick_drivers.git)
  * [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy.git)

Selected features from previous Alpha/Beta releases (for the complete list, see the [[earlier release notes|Releases]]):
* C++ and Python implementations of ROS 2 client libraries including APIs for:
  * Publishing and subscribing to ROS topics
  * Requesting and replying ROS services (synchronous (C++ only) and asynchronous)
  * Getting and setting ROS parameters (C++ only, synchronous and asynchronous)
  * Timer callbacks
* Support for interoperability between multiple DDS/RTPS implementations
  * eProsima FastRTPS is our default implementation, and is included in the binary packages
  * RTI Connext is supported: build from source to try it out
  * We initially supported PrismTech OpenSplice but support for it is currently on hold
* A graph API for network events
* Distributed discovery
* Realtime safe code paths for publish and subscribe with compatible DDS implementation (only Connext at the moment)
  * Support for custom allocators
* ROS 1 <-> ROS 2 dynamic bridge node
* Executor threading model (C++ only)
* Component model to compose nodes at compile / link / runtime
* Managed component using a standard lifecycle
* Extended `.msg` format with new features:
  * Bounded arrays
  * Default values

Known issues:
* We’re tracking issues in various repositories, but the main entry point is the [ros2/ros2 issue tracker](https://github.com/ros2/ros2/issues)
* We’d like to highlight a [known issue](https://github.com/ros2/rmw_connext/issues/234) that we are looking into which doesn't allow two topics with the same base name but different namespaces to have a different type when using `rmw_connext_cpp`.
* Services with long responses are not working with Fast-RTPS. The fix, while not being part of beta2, is available upstream so you can work around this issue by building from source using Fast-RTPS master branch.