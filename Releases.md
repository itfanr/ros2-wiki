A summary of releases of ROS 2 software is listed below. For more details about each release, see the corresponding release overview. For convenience the key features of each release are repeated on this page.

| Release Overview | Date |
| --- | --- |
| [[alpha8|alpha8-Overview]] | 4 October 2016 |
| [[alpha7|alpha7-Overview]] | 11 July 2016 |
| [[alpha6|alpha6-Overview]] | 1 June 2016 |
| [[alpha5|alpha5-Overview]] | 5 April 2016 |
| [[alpha4|alpha4-Overview]] | 17 February 2016 |
| [[alpha3|alpha3-Overview]] | 18 December 2015 |
| [[alpha2|alpha2-Overview]] | 3 November 2015 |
| [[alpha1|alpha1-Overview]] | 31 August 2015 |

Notes on how an alpha release is made: [[Release-Howto]]


# New features of each release


## [[alpha8|alpha8-Overview]]
- Several improvements to Fast RTPS and its rmw implementation
  - Support for large (image) messages in Fast RTPS
  - `wait_for_service` functionality in Fast RTPS
- Support for all ROS 2 message types in Python and C
- Added support for Quality of Service (QoS) settings in Python
- Fixed various bugs with the previous alpha release


## [[alpha7|alpha7-Overview]]
- Graph API functionality: wait_for_service
  - Added interfaces in rclcpp and make use of them in examples, demos, and tests.
- Improved support for large messages in both Connext and Fast-RTPS (partial for Fast-RTPS).
- Turtlebot demo using ported code from ROS 1.
  - See: https://github.com/ros2/turtlebot2_demo


## [[alpha6|alpha6-Overview]]
- Graph API functionality: wait_for_service.
  - Added graph guard condition to nodes for waiting on graph changes.
  - Added `rmw_service_server_is_available` for verifying a service is available.
- Refactored `rclcpp` to use `rcl`.
- Improved support for complex message types in Python.
  - Nested messages
  - Arrays
  - Strings


## [[alpha5|alpha5-Overview]]
- Support for C data structures in Fast RTPS and Connext Dynamic rmw implementations.
- Support services in C.
- Added 32-bit and 64-bit ARM as experimentally supported platforms.


## [[alpha4|alpha4-Overview]]
- Improved type support infrastructure, including support for C.
- Preliminary Python client library, only publishers and subscriptions are supported.
- Added structures for ROS time in C API (still needs C++ API).
  - New concept of extensible "time sources" for ROS Time, the default time source will be like ROS 1 (implementation pending).

## [[alpha3|alpha3-Overview]]
- Updated `rcl` interface.
 - This interface will be wrapped in order to create language bindings, e.g. `rclpy`.
 - This interface has improved documentation and test coverage over existing interfaces we currently have, e.g. `rmw` and `rclcpp`.
 - See: https://github.com/ros2/rcl/tree/release-alpha3/rcl/include/rcl
- Added support in rclcpp for using the TLSF (two-level segregate fit) allocator, a memory allocator design for embedded and real-time systems.
- Improved efficiency of MultiThreadedExecutor and fixed numerous bugs with multi-threaded execution, which is now test on CI.
- Added ability to cancel an Executor from within a callback called in spin.
- Added ability for a timer to cancel itself by supporting a Timer callback that accepts a reference to itself as a function parameter.
- Added checks for disallowing multiple threads to enter Executor::spin.
- Improved reliability of numerous tests that had been sporadically failing.
- Added support for using FastRTPS (instead of, e.g., OpenSplice or Connext).
- A partial port of tf2 including the core libraries and core command line tools. 

## [[alpha2|alpha2-Overview]]
- Support for custom allocators in rclcpp, useful for real-time messaging.
- Feature parity of Windows with Linux/OSX, including workspace management, services and parameters.
- rclcpp API improvements.
- FreeRTPS improvements.

## [[alpha1|alpha1-Overview]]
- Discovery, transport, and serialization [use DDS](http://design.ros2.org/articles/ros_on_dds.html).
- Support [multiple DDS vendors](http://design.ros2.org/articles/ros_on_dds.html#vendors-and-licensing).
- Support messaging primitives: topics (publish / subscribe), services (request / response), and parameters.
- Support Linux (Ubuntu Trusty), OS X (Yosemite) and Windows (8).
- [[Use quality-of-service settings to handle lossy networks|Quality-Of-Service]].
- [[Communicate inter-process or intra-process with the same API|Intra-Process-Communication]].
- [[Write real-time safe code that uses the ROS 2 APIs|Real-Time-Programming]].
- [Run ROS 2 on "bare-metal" microcontrollers (no operating system)](https://github.com/ros2/freertps/wiki).
- [[Bridge communication between ROS 1 and ROS 2|https://github.com/ros2/ros1_bridge/blob/master/README.md]].