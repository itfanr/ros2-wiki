# Installing ROS 2 on Linux

This page explains how to install ROS 2 on Linux from a pre-built binary package.

## System Requirements

We support Ubuntu Linux Trusty Tahr 14.04 on 64-bit x86 (no binaries available yet for ARM or 32-bit x86).

## Installing prerequisites

1. Install the OSRF sources for DDS via debian package:

        apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
        sudo bash -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
1. Install runtime dependencies and wget:

        sudo apt-get update && sudo apt-get install -q -y \
            libopencv-core2.4 \
            libhighgui2.4 \
            libopencv-imgproc2.4 \
            libopensplice64 \
            wget
1. *Optional*: if you want to use the ROS 1<->2 bridge, then you must also install ROS 1.
  Follow the normal install instructions: http://wiki.ros.org/indigo/Installation/Ubuntu

## Downloading ROS 2

* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for Linux; let's assume that it ends up at `~/Downloads/ros2-package-linux.tar.bz2`.
* Unpack it:

        mkdir -p ~/ros2_install
        cd ~/ros2_install
        tar xf ~/Downloads/ros2-package-linux.tar.bz2

## Try some examples

In one terminal, source the setup file and then run a `talker`:

    . ~/ros2_install/ros2-linux/setup.bash
    talker
In another terminal source the setup file and then run a `listener`:

    . ~/ros2_install/ros2-linux/setup.bash
    listener
You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

### ROS 1 bridge

If you have ROS 1 installed, you can try the ROS 1 bridge, by first sourcing your ROS 1 setup file; we'll assume that it's `/opt/ros/indigo/setup.bash`.

If you haven't already, start a roscore:

    . /opt/ros/indigo/setup.bash
    roscore

In another terminal, start the bridge:

    . /opt/ros/indigo/setup.bash
    . ~/ros2_install/ros2-linux/setup.bash
    dynamic_bridge
For more information on the bridge, read the [tutorial](https://github.com/ros2/ros1_bridge/blob/master/README.md).
