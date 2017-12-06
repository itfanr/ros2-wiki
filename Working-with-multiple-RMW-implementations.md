# Working with multiple ROS 2 middleware implementations
This page explains the default RMW implementation and how to specify an alternative.

## Pre-requisites
You should have already read the [[DDS and ROS middleware implementations]] page.

## Multiple RMW implementations
The current ROS 2 binary releases only support one implementation at a time (FastRTPS, indicated by their file name) since none of the other supported RMW implementations can be distributed freely.

While the ROS 2 binary releases only include one RMW implementation each, a ROS 2 workspace that has been built from source may build and install multiple RMW implementations simultaneously.
While the core ROS 2 code is being compiled, any RMW implementation that is found will be built if the relevant DDS implementation has been installed properly and the relevant environment variables configured.
For example, if the code for the [RMW package for Fast RTPS](https://github.com/ros2/rmw_fastrtps_cpp) is in the workspace, it will be built if a Fast RTPS installation can also be found.
For many cases you will find that nodes using different RMW implementations are able to communicate, however this is not true under all circumstances.
A list of supported inter-vendor communication configurations is forthcoming.

## Default RMW implementation

If a ROS 2 workspace has multiple RMW implementations, the default RMW implementation is currently selected as Fast RTPS if it's available.
If the Fast RTPS RMW implementation is not installed, the RMW implementation with the first RMW implementation identifier in alphabetical order will be used.
The implementation identifier is the name of the ROS package that provides the RMW implementation, e.g. `rmw_fastrtps_cpp`.
For example, if both `rmw_opensplice_cpp` and `rmw_connext_cpp` ROS packages are installed, `rmw_connext_cpp` would be the default.
If `rmw_fastrtps_cpp` is ever installed, it would be the default.
See below for how to specify which RMW implementation is to be used when running the ROS 2 examples.

*Note:* for ROS 2 alpha releases up to and including alpha 8, only the 'alphabetical order' rule explained above is used, and so the RMW implementation for Fast RTPS does not have any explicit priority over other RMW implementations.

## Specifying RMW implementations

---

To have multiple RMW implementations available for use you must have built from source (see above).

---

### C++

The ROS 2 C++ demos ([demo_nodes_cpp](https://github.com/ros2/demos/tree/master/demo_nodes_cpp/src)) are building a set of executables, e.g. `talker`.
By default these will use the default RMW implementation.
To choose a different RMW implemenation you can set the environment variable `RMW_IMPLEMENTATION` to a specific implementation identifier, e.g.:

- `RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_cpp talker` which will run the talker demo with the C++ RMW implementation for Connext.

#### In beta 1 and earlier

In the beta 1 and earlier releases the `RMW_IMPLEMENTATION` environment variable was not yet supported.
Instead the ROS 2 C++ demos ([demo_nodes_cpp](https://github.com/ros2/demos/tree/master/demo_nodes_cpp/src)) are configured to build a version of each executable for all RMW implementations that have been built.
For example, for the simple "talker" publisher demo, if you have both Fast RTPS and Connext RMW implementations installed, the following executables are created:

- a C++ executable named `talker__rmw_fastrtps_cpp` which uses a C++ RMW implementation for Fast RTPS
- a C++ executable named `talker__rmw_connext_cpp` which uses a C++ RMW implementation for Connext
- a C++ executable named `talker` which uses the default RMW implementation

### Python

The ROS 2 Python examples ([demo_nodes_py](https://github.com/ros2/demos/tree/master/demo_nodes_py)) will generate one executable Python script each that will by default use the default RMW implementation.
For example, assuming that you have installed and setup your ROS 2 workspace, the following invocation will run the Python simple "talker" publisher demo with the default RMW implementation:

```bash
ros2 run demo_nodes_py talker
```

The same way as for C++ the RMW implementation can be chosen with the environment variable `RMW_IMPLEMENTATION`, e.g.:

```bash
export RMW_IMPLEMENTATION=rmw_connext_cpp
ros2 run demo_nodes_py talker
```
#### In beta 1 and earlier

In the beta 1 and earlier releases the `RMW_IMPLEMENTATION` environment variable was not yet supported.
Instead the "talker" demo can also be configured to use a particular RMW implementation through the use of the `RCLPY_IMPLEMENTATION` environment variable.
For example, to specify that the `rmw_fastrtps_cpp` RMW implementation is to be used, on Linux you can run:

```bash
RCLPY_IMPLEMENTATION=rmw_fastrtps_cpp talker_py
```

## Adding RMW implementations to your workspace

Suppose that you have built your ROS 2 workspace with only Fast RTPS installed and therefore only the Fast RTPS RMW implementation built.
The last time your workspace was built, any other RMW implementation packages, `rmw_connext_cpp` for example, were probably unable to find installations of the relevant DDS implementations.
If you then install an additional DDS implementation, Connext for example, you will need to re-trigger the check for a Connext installation that occurs when the Connext RMW implementation is being built.
You can do this by specifying the `--force-cmake-configure` flag on your next workspace build, and you should see that the RMW implementation package then gets built for the newly installed DDS implementation.

It is possible to run into a problem when "rebuilding" the workspace with an additional RMW implementation using the `--force-cmake-configure` option where the build complains about the default RMW implementation changing.
To resolve this, you can either set the default implementation to what is was before with the `RMW_IMPLEMENTATION` CMake argument or you can delete the build folder for packages that complain and continue the build with `--start-with <package name>`.

## Troubleshooting

### Ensuring use of a particular RMW implementation

#### ROS 2 Ardent and later

If the `RMW_IMPLEMENTATION` environment variable is set to an RMW implementation for which support is not installed, you will see an error message similar to the following:
```
Expected RMW implementation identifier of 'rmw_connext_cpp' but instead found 'rmw_fastrtps_cpp', exiting with 102.
```

If this occurs, double check that your ROS 2 installation includes support for the RMW implementation that you have specified in the `RMW_IMPLEMENTATION` environment variable.

#### ROS 2 beta 2 and later

In ROS 2 beta 2 / beta 3, specifying an invalid RMW implementation does not cause an error if support for only one implementation is installed.
If you want to be certain that a particular RMW implementation is being used, you can set the `RCL_ASSERT_RMW_ID_MATCHES` environment variable, which will only allow nodes to be created with that RMW implementation.

```
RCL_ASSERT_RMW_ID_MATCHES=rmw_connext_cpp RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_cpp talker
```