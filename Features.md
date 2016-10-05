# ROS 2 Feature Status

The features listed below are available in the current ROS 2 alpha release.
Unless otherwise specified, the features are available for all supported platforms (Ubuntu 16.04, OS X 10.10+, Windows 10), DDS implementations (eProsima Fast RTPS and RTI Connext) and programming language client libraries (C++ and Python).
For planned future development, see the [[Roadmap]].

| Functionality | Link | Fine print |
| --- | --- | --- |
| Discovery, transport and serialization over DDS | [Article](http://design.ros2.org/articles/ros_on_dds.html) | |
| Support for multiple DDS implementations | | Currently eProsima Fast RTPS and RTI Connext. |
| Publish/subscribe over topics | [Sample code](https://github.com/ros2/examples) | |
| Clients and services | [Sample code](https://github.com/ros2/examples) | Services not yet available in Python. "wait_for_service" not available for RTI Connext. |
| Set/retrieve parameters | [Sample code](https://github.com/ros2/examples) | Parameters not yet available in Python. |
| ROS 1 - ROS 2 communication bridge | [Tutorial](https://github.com/ros2/ros1_bridge/blob/master/README.md) | Only for topics, not yet available for services, actions, etc. |
| Quality of service settings for handling non-ideal networks | [[Demo|Quality-Of-Service]] | |
| Inter- and intra-process communication using the same API | [[Demo|Intra-Process-Communication]] | Currently only in C++, not Python. |
| Preliminary support for real-time code | [[Demo|Real-Time-Programming]], [[demo|Allocator-Template-Tutorial]] | Linux only. Not available for Fast RTPS. |
| Preliminary support for "bare-metal" microcontrollers | [Wiki](https://github.com/ros2/freertps/wiki)| |
