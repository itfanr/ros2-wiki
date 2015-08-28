
## Overview

This will provide you with a quick summary of how to get up and running using an ament workspace.
It will be a practical tutorial and is not designed to replace the core documentation.

## Background

Ament is an interation on the catkin meta-build tool.
For more information on the design of ament see [this document](http://design.ros2.org/articles/ament.html).

The source can be found in [ament github orgainization](https://github.com/ament).

## Prerequisites

## Basics

An ament workspace is a directory with a particular structure.
The one requirement is that there is a `src` subdirectory.
Inside that subdirectory is where the sourecode will be located.
Typically the directory starts otherwise empty.

Ament does out of source builds.
By default it will create a `build` and `install` directory as peers of the `src` directory.
The `build` directory will be where intermediate files are stored.
The `install` directory is where the resultant libraries and executables will be generated.

NB: Compared to catkin there is no `devel` directory.

## Create directory structure

To make the basic structure in the directory `~/example_ament_ws`:

```bash
mkdir ~/ros2_ws
mkdir ~/ros2_ws/src
cd ~/ros2_ws
```


## Add some sources

To start off we need to setup a underlay without any of ROS2 installed.

```
wget https://raw.githubusercontent.com/ros2/examples/master/ros2.repos
vcs import ~/ros2_ws/src < ros2.repos
```

## Run the build

Since this is a bootstrap environment we need to call ament.py by it's full path.

Note: In the future once ament is either installed on your system or in an underlayed workspace this will no longer be necessary.

Since there is not `devel` space in ament it supports the option for `--symlink-install` for faster scripting language iteration.

```
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
```

## Run the unit tests

Ament has unit testing and system tests integrated.
To run the unit tests you just built, with the `--build-tests` option above, run the following.

```
src/ament/ament_tools/scripts/ament.py test --skip-build
```

## Source the environment

When ament has completed building successfully the output will be in the `install` directory.
To use the output libraries and executables you need add the `install` directory to your path.
Ament will have generated bash files in the `install` directory to help do this.
These files will both add the required elements to your path and library paths as well as provide any exported bash or shell commands exported by packages.

```
. install/local_setup.bash
```

NB: This is slightly different than catkin.
The `local_setup.*` file is slightly different that the `setup.*` file in that it will only apply settings from the current workspace.
`setup.*` will recursively set parameters.

## Try a demo

With the environment sourced you can now run executables built by ament.

```
listener &
talker
```

And you will see the numbers incrementing.

Lets take down the nodes and try creating our own workspace overlay.
```
^-C
kill %1
```

## Develop your own package

Ament uses the same package.xml as catkin defined in [REP 140](http://www.ros.org/reps/rep-0140.html).

You can create your own package inside the `src` directory however it is recommended to use an overlay when

## Create an overlay

Now that you have setup your bootstrap underlay you will also find `ament` is on your path.

Lets make a new overlay directory `~/overlay_ws`.

```
mkdir -p ~/overlay_ws/src
cd ~/overlay_ws/src
```

And to get started we'll overlay the [ros2/examples repository](https://github.com/ros2/examples)

```
git clone https://github.com/ros2/examples.git
```

And build the overlay, but lets build with debug so we can make sure to get :

```
cd ~/overlay_ws
ament build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Now this overlay is on top of the existing overlay so you'll find that `which talker` currently refers to the one from the underlay.

If you source `~/overlay_ws/install/local_setup.bash` it will change to refer to talker in the overlay.

If you are returning with a new terminal to your development and want to pick up developing on your overlay you can simply source `~/overlay_ws/setup.bash` which will source all parent workspaces environments automatically.

## Create your own package

You can create your own package.
The equivalent of catkin_create_package will be ported to ament but is not available yet.

Ament supports multiple build types.
The recommended build types are `ament_cmake` and `ament_python`.
Also implements is pure `cmake` packages.
It's expected to add support for more [build types](https://github.com/ament/ament_tools/blob/master/doc/development/build_types.rst).

An example of an `ament_python` build is the []`ament_tools` package](https://github.com/ament/ament_tools), where the setup.py is the primary entry point for building.

While a package such as [`rclcpp_examples`](https://github.com/ros2/examples/tree/master/rclcpp_examples) uses the `ament_cmake` build type, and uses CMake as the build tool.

## Tips

If you do not want to build a specific package place an empty file named `AMENT_IGNORE` in the directory and it will not be indexed.