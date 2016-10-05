# ROS 2 and different DDS vendors
[ROS 2 is built on top of DDS as its middleware](http://design.ros2.org/articles/ros_on_dds.html), which it uses for discovery, serialization and transportation.
DDS is an industry standard which is then implemented by a range of vendors, such as RTI's implementation [Connext](https://www.rti.com/products/) and eProsima's implementation [Fast RTPS](http://www.eprosima.com/index.php/products-all/eprosima-fast-rtps).
ROS 2 supports multiple DDS implementations as it is not necessarily "one size fits all" when it comes to vendors/implementations.
Reasons you might take into consideration while choosing a DDS implementation include the license in addition to various technical considerations of the implementation.
Even a single vendor may provide multiple DDS implementations: for example, RTI has variations of Connext that range from specifically targeting microcontrollers to targeting applications requiring flight-certification (we support the standard desktop version).

In order to use a DDS implementation with ROS 2, a ROS middleware (RMW) package needs to be created that implements the abstract ROS middleware interface using the DDS implementation's API and tools.
It's a lot of work to implement and maintain RMW packages for supporting DDS implementations, but supporting at least a few implementations is important for ensuring that the ROS 2 codebase is not tied to any one in particular, as users may wish to switch out implementations depending on their project's needs.

The [ROS 2 binary releases](https://github.com/ros2/ros2/releases/) specify in their name the one RMW implementation that is included in the binary.

While the ROS 2 binary releases only include one RMW implementation each, a ROS 2 workspace that has been built from source may have multiple RMW implementations installed.
While the core ROS 2 code is being compiled, any RMW implementation that is found will be built if the relevant DDS implementation has been installed properly.
For example, if the code for the [RMW package for Fast RTPS](https://github.com/eProsima/ROS-RMW-Fast-RTPS-cpp) is in the workspace, it will be built if a Fast RTPS installation can also be found.
If there are multiple RMW implementations available, other ROS 2 packages that require an RMW implementation will then have to specify which RMW implementation is to be used.
For many cases you will find that nodes using different RMW implementations are able to communicate, though it this is not true under all circumstances.

# Default RMW implementation
If a ROS 2 workspace has multiple RMW implementations, the default RMW implementation is currently selected as the first in alphabetical order.
For example, if both `rmw_fastrtps_cpp` and `rmw_connext_cpp` RMW implementations are installed, `rmw_connext_cpp` would be the default.
See below for how to specify which RMW implementation is to be used when running the ROS 2 examples.

# Specifying RMW implementations
## C++
The ROS 2 C++ examples are then configured to build executables for each RMW implementation that has been built.
For example, for the simple "talker" publisher demo, if you have both Fast RTPS and Connext RMW implementations installed, the following executables are created:
- a C++ executable named `talker__rmw_fastrtps_cpp` which uses a C++ RMW implementation for Fast RTPS
- a C++ executable named `talker__rmw_connext_cpp` which uses a C++ RMW implementation for Connext
- a C++ executable named `talker` which uses the default RMW implementation

If you are curious about how an executable can be automatically created for each RMW implementation for the above example, it's through [the use of](https://github.com/ros2/examples/blob/release-alpha8/rclcpp_examples/CMakeLists.txt#L59) the `call_for_each_rmw_implementation` macro provided by the [RMW CMake package](https://github.com/ros2/rmw/tree/release-alpha8/rmw_implementation_cmake).

## Python
The ROS 2 Python examples will generate one executable Python script that will use the default RMS implementation.
For example, assuming that you have installed and setup your ROS 2 workspace, the following invocation will run the Python simple "talker" publisher demo with the default RMW implementation:
```
python3 `which talker_py`
```
The "talker" demo can also be configured to use a particular RMW implementation through the use of the `RCLPY_IMPLEMENTATION` environment variable.
For example, to specify that the `rmw_fastrtps_cpp` RMW implementation is to be used, you can run:
```
RCLPY_IMPLEMENTATION=rmw_fastrtps_cpp python3 `which talker_py`
```

# Adding RMW implementations to your workspace
Suppose that you have built your ROS 2 workspace with only Fast RTPS installed and therefore only the Fast RTPS RMW implementation built.
The last time your workspace was built, any other RMW implementation packages, `rmw_connext_cpp` for example, were probably unable to find installations of the relevant DDS implementations.
If you then install an additional DDS implementation, Connext for example, you will need to re-trigger the check for a Connext installation that occurs when the Connext RMW implementation is being built.
You can do this by specifying the `--force-cmake-configure` flag on your next workspace build, and you should see that the RMW implementation package then gets built for the newly installed DDS implementation.