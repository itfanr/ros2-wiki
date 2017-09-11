# Installing ROS 2 on OS X

This page explains how to install ROS 2 on OS X from a pre-built binary package.

## System requirements

We support OS X El Capitan and Sierra (10.11.x and 10.12.x).

## Installing prerequisites

You need the following things installed before installing ROS 2.

 1. **brew** *(needed to install more stuff; you probably already have this)*:
  * Follow installation instructions at http://brew.sh/
  * *Optional*: Check that `brew` is happy with your system configuration by running:

         brew doctor
      Fix any problems that it identifies.
 1. Add some extra sources of software for `brew`:

        brew tap osrf/ros2
        brew tap ros/deps
 1. Use `brew` to install more stuff:

        brew install python3

        # install asio and tinyxml2 for Fast-RTPS
        brew install asio tinyxml2

        # install dependencies for robot state publisher
        brew install tinyxml eigen pcre

        # OpenCV isn't a dependency of ROS 2, but it used by some demos.
        # We're disabling python support in opencv to avoid a dependency on numpy,
        # which in turn will want to build gcc 5.2.0, which takes a long time.
        brew install homebrew/science/opencv --without-python

 1. Install additional runtime dependencies for command-line tools:

        python3 -m pip install pyyaml setuptools argcomplete

## Downloading ROS 2

- Go the releases page: https://github.com/ros2/ros2/releases
- Download the latest package for OS X; let's assume that it ends up at `~/Downloads/ros2-package-osx-x86_64.tar.bz2`.
  - Note: there may be more than one binary download option which might cause the file name to differ.
- Unpack it:

        mkdir -p ~/ros2_install
        cd ~/ros2_install
        tar xf ~/Downloads/ros2-package-osx-x86_64.tar.bz2

## Try some examples

In one terminal, source the setup file and then run a `talker`:

    . ~/ros2_install/ros2-osx/setup.bash
    ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a `listener`:

    . ~/ros2_install/ros2-osx/setup.bash
    ros2 run demo_nodes_cpp listener

_For Beta 1 releases and earlier, invoke the executables directly by calling "`talker`" and "`listener`"._

You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

If you run into issues, see the troubleshooting section on the source installation page: https://github.com/ros2/ros2/wiki/OSX-Development-Setup#troubleshooting
