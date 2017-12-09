# Installing ROS 2 on OS X

This page explains how to install ROS 2 on OS X from a pre-built binary package.

## System requirements

We support OS X El Capitan and Sierra (10.11.x and 10.12.x).

## Installing prerequisites

You need the following things installed before installing ROS 2.

- **brew** *(needed to install more stuff; you probably already have this)*:
  - Follow installation instructions at http://brew.sh/
  - *Optional*: Check that `brew` is happy with your system configuration by running:

         brew doctor
      Fix any problems that it identifies.

- Use `brew` to install more stuff:

        brew install python3

        # install asio and tinyxml2 for Fast-RTPS
        brew install asio tinyxml2

        # install dependencies for robot state publisher
        brew install tinyxml eigen pcre

        # OpenCV isn't a dependency of ROS 2, but it is used by some demos.
        brew install opencv

        # install OpenSSL for DDS-Security
        brew install openssl

        # install Qt for RViz
        brew install qt freetype assimp

- Install additional runtime dependencies for command-line tools:

        python3 -m pip install pyyaml setuptools argcomplete


## Disable System Integrity Protection (SIP)
OS X versions >=10.11 have System Integrity Protection enabled by default.
So that SIP doesn't prevent processes from inheriting dynamic linker environment variables, such as `DYLD_LIBRARY_PATH`, you'll need to disable it [following these instructions](https://developer.apple.com/library/content/documentation/Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html).

## Downloading ROS 2

- Go the releases page: https://github.com/ros2/ros2/releases
- Download the latest package for OS X; let's assume that it ends up at `~/Downloads/ros2-package-osx-x86_64.tar.bz2`.
  - Note: there may be more than one binary download option which might cause the file name to differ.
- Unpack it:

        mkdir -p ~/ros2_install
        cd ~/ros2_install
        tar xf ~/Downloads/ros2-package-osx-x86_64.tar.bz2

- If you downloaded a package that includes support for OpenSplice, you will need to install OpenSplice as it is not included in the package itself.
Download the latest release from https://github.com/ADLINK-IST/opensplice/releases and unpack it.

## Set up the ROS 2 environment

Source the ROS 2 setup file:

    . ~/ros2_install/ros2-osx/setup.bash

If you downloaded a release with OpenSplice support you must additionally source the OpenSplice setup file.
Only do this **after** you have sourced the ROS 2 one:

    . <path_to_opensplice>/x86_64.darwin10_clang/release.com


## Try some examples

In one terminal, set up the ROS 2 environment as described above and then run a `talker`:

    ros2 run demo_nodes_cpp talker

In another terminal, set up the ROS 2 environment and then run a `listener`:

    ros2 run demo_nodes_cpp listener

_For Beta 1 releases and earlier, invoke the executables directly by calling "`talker`" and "`listener`"._

You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

If you run into issues, see the troubleshooting section on the source installation page: https://github.com/ros2/ros2/wiki/OSX-Development-Setup#troubleshooting
