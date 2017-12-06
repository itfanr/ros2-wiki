# Installing ROS 2 on Linux

This page explains how to install ROS 2 on Linux from a pre-built binary package.

As of Beta 2 there are also [[Debian packages|Linux-Install-Debians]] for Ubuntu Xenial available.

## System Requirements

We support Ubuntu Linux Xenial Xerus 16.04 on 64-bit x86 (no binaries available yet for ARM or 32-bit x86).

Note: alpha versions 6 and earlier supported Ubuntu Trusty Tahr 14.04.

## Installing prerequisites

1. Install runtime dependencies and wget:

        sudo apt-get update && sudo apt-get install -q -y \
            libopencv-core2.4v5 \
            libhighgui2.4 \
            libopencv-imgproc2.4v5 \
            libasio-dev \
            libeigen3-dev \
            libtinyxml-dev \
            libtinyxml2-dev \
            libcurl4-openssl-dev \
            libqt5core5a \
            libqt5gui5 \
            libqt5opengl5 \
            libqt5widgets5 \
            libxaw7-dev \
            libgles2-mesa-dev \
            libglu1-mesa-dev \
            python3-yaml \
            python3-setuptools \
            wget

1.  If you are installing beta-1 or older, you also need boost:

        sudo apt-get update && sudo apt-get install -q -y \
            libboost-thread-dev

1.  Install argcomplete from pip for argument completion.  Note that the version available through apt-get in Ubuntu 16.04 (Xenial) will not work due to a bug in argcomplete:

        sudo pip3 install argcomplete

1. *Optional*: if you want to use the ROS 1<->2 bridge, then you must also install ROS 1.
  Follow the normal install instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu

1. *Optional*: install a Debian package of PrismTech OpenSplice built by OSRF:

        sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
        sudo bash -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
        sudo apt-get update && sudo apt-get install -q -y \
            libopensplice67

  To use OpenSplice you need to set the environment variable `RMW_IMPLEMENATION` accordingly (otherwise it will default to use FastRTPS):

        export RMW_IMPLEMENATION=rmw_opensplice_cpp

## Downloading ROS 2

* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for Linux; let's assume that it ends up at `~/Downloads/ros2-package-linux-x86_64.tar.bz2`.
  * Note: there may be more than one binary download option which might cause the file name to differ.
* Unpack it:

        mkdir -p ~/ros2_install
        cd ~/ros2_install
        tar xf ~/Downloads/ros2-package-linux-x86_64.tar.bz2

## Try some examples

In one terminal, source the setup file and then run a `talker`:

    . ~/ros2_install/ros2-linux/setup.bash
    ros2 run demo_nodes_cpp talker
In another terminal source the setup file and then run a `listener`:

    . ~/ros2_install/ros2-linux/setup.bash
    ros2 run demo_nodes_cpp listener
You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

See the [demos](Tutorials) for other things to try, including how to [run the talker-listener example in Python](Python-Programming).

### ROS 1 bridge

If you have ROS 1 installed, you can try the ROS 1 bridge, by first sourcing your ROS 1 setup file; we'll assume that it's `/opt/ros/kinetic/setup.bash`.

If you haven't already, start a roscore:

    . /opt/ros/kinetic/setup.bash
    roscore

In another terminal, start the bridge:

    . /opt/ros/kinetic/setup.bash
    . ~/ros2_install/ros2-linux/setup.bash
    ros2 run ros1_bridge dynamic_bridge
For more information on the bridge, read the [tutorial](https://github.com/ros2/ros1_bridge/blob/master/README.md).
