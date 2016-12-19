# ROS 2 and different DDS vendors

ROS 2 is built on top of DDS as its middleware, which it uses for discovery, serialization and transportation.
[This article](http://design.ros2.org/articles/ros_on_dds.html) explains the motivation behind using DDS in detail, but in summary DDS is an end-to-end middleware that provides features which are relevant to ROS systems, such as distributed discovery (not centralized like in ROS 1) and control over different "Quality of Service" options for the transportation.

DDS is an industry standard which is then implemented by a range of vendors, such as RTI's implementation [Connext](https://www.rti.com/products/) and eProsima's implementation [Fast RTPS](http://www.eprosima.com/index.php/products-all/eprosima-fast-rtps).
ROS 2 supports multiple DDS implementations because it is not necessarily "one size fits all" when it comes to choosing a vendor/implementation.
There are many factors you might consider while choosing a DDS implementation: logistical considerations like the license or technical considerations like platform availability or computation footprint.
Vendors may provide more than one DDS implementation targeted at meeting different needs.
For example, RTI has a few variations of their Connext implementation that range in purpose, from specifically targeting microcontrollers to targeting applications requiring special certifications (we support the standard desktop version).

In order to use a DDS implementation with ROS 2, a ROS middleware (RMW) package needs to be created that implements the abstract ROS middleware interface using the DDS implementation's API and tools.
It's a lot of work to implement and maintain RMW packages for supporting DDS implementations, but supporting at least a few implementations is important for ensuring that the ROS 2 codebase is not tied to any one particular implementation, as users may wish to switch out implementations depending on their project's needs.

### Supported RMW implementations

| Product name | License | RMW implementation | Status |
| ------------- | ------------- | ----- | ---- | --- |
| eProsima _Fast RTPS_ | Apache 2 | `rmw_fastrtps_cpp` | Full support. Default RMW. Packaged with binary releases. |
| RTI _Connext_ | commercial, research | `rmw_connext_cpp` | Full support. Building from source required. |
| RTI _Connext_ (dynamic implementation) | commercial, research | `rmw_connext_dynamic_cpp` | Support paused. Full support until alpha 8.* |
| PrismTech _Opensplice_ | LGPL (only v6.4), commercial | `rmw_opensplice_cpp` | Support paused. Full support until alpha 8.* |
| OSRF _FreeRTPS_ | Apache 2 | -- | Partial support. Development paused. |


_*Support paused means that, since the alpha 8 release of ROS 2, features that have been added to ROS 2 have not been added to these middleware implementations.
These middleware implementations may or may not be in a functional state._

For practical information on working with multiple RMW implementations, see the [[Working with multiple RMW implementations]]. page