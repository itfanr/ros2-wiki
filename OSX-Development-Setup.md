# Building ROS 2 on OS X

## System requirements

We support OS X Yosemite (10.10.x).

## Install prerequisites

**TODO: Extend these instructions for other DDS implementations, starting with RTI Connext.**

You need the following things installed to build ROS 2:

 1. **Java Development Kit (JDK)** *(currently required to compile the OpenSplice DDS implementation, so you can skip this step if you're not going to use OpenSplice; also that requirement might go away in the future, e.g., if we disable building their Java bindings)*:
  * Go to http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html
  * Accept the license terms and download the "Mac OS X x64" version of the `.dmg` file.
  * Install from the `.dmg`.
  * *Optional*: check that you have a `jni.h` and that your version of `java` is 1.8:

            $ find /Library/Java | grep jni.h
            /Library/Java/JavaVirtualMachines/jdk1.8.0_60.jdk/Contents/Home/include/jni.h
            $ java -version
            java version "1.8.0_60"
            Java(TM) SE Runtime Environment (build 1.8.0_60-b27)
            Java HotSpot(TM) 64-Bit Server VM (build 25.60-b23, mixed mode)
 1. **brew** *(needed to install more stuff; you probably already have this)*:
  * Follow installation instructions at http://brew.sh/
  * *Optional*: Check that `brew` is happy with your system configuration by running:

            brew doctor
        Fix any problems that it identifies.
 1. Add some extra sources of software for `brew`:

        brew tap osrf/ros2
        brew tap ros/deps
 1. Use `brew` to install more stuff:

        brew install python3 wget cmake cppcheck gtest

        # install boost for FastRTPS
        brew install boost

        # install dependencies for robot state publisher
        brew install tinyxml eigen

        # if you're going to build for OpenSplice, install it
        brew install opensplice

        # We're disabling python support in opencv to avoid a dependency on numpy,
        # which in turn will want to build gcc 5.2.0, which takes a long time.
        brew tap homebrew/science
        brew install opencv --without-python
    *Optional*: Make sure that you have `gtest` 1.7:

        $ brew info gtest
        ros/deps/gtest: stable 1.7.0
        http://code.google.com/p/googletest/
        /usr/local/Cellar/gtest/1.7.0 (32 files, 1.2M) *
          Built from source
        From: https://github.com/ros/homebrew-deps/blob/master/gtest.rb
 1. Use `python3 -m pip` (**not `pip`**, which may install Python 2.7 packages!) to install more stuff:

        python3 -m pip install empy setuptools nose vcstool pep8 pydocstyle pyflakes flake8 mock coverage
 1. *Optional*: if you want to build the ROS 1<->2 bridge, then you must also install ROS 1:
  * Start with the normal install instructions: http://wiki.ros.org/indigo/Installation/OSX/Homebrew/Source
  * When you get to the step where you call `rosinstall_generator` to get the source code, here's an alternate invocation that brings in just the minimum required to produce a useful bridge:

            rosinstall_generator catkin common_msgs roscpp rosmsg --rosdistro indigo --deps --wet-only --tar > indigo-ros2-bridge-deps.rosinstall
            wstool init -j8 src indigo-ros2-bridge-deps.rosinstall
    Otherwise, just follow the normal instructions, then source the resulting `install_isolated/setup.bash` before proceeding here to build ROS 2.

## Get the ROS 2 code

Create a workspace and clone all repos:

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
    vcs import src < ros2.repos

This will get the code for the latest ROS 2 release. If you want the code from a particular release or from the development branches, see [this page](Maintaining-a-Source-Checkout).

## Build the ROS 2 code

**Note**: if you installed and sourced the setup file for ROS 1, then you should instead follow the [modified instructions](#building-with-the-ros-1-bridge).

Run the `ament` tool to build everything (more on using `ament` in [[this tutorial|Ament-Tutorial]]):

    cd ~/ros2_ws/
    src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install

### Building with the ROS 1 bridge

To build the ROS 1 bridge, read the [ros1_bridge tutorial](https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source).

## Try some examples

In one terminal, source the setup file and then run a `talker`:

    . ~/ros2_ws/install/setup.bash
    talker

In another terminal source the setup file and then run a `listener`:

    . ~/ros2_ws/install/setup.bash
    listener

You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

## Maintain your source checkout
For information on how to keep your source checkout up-to-date, see [Maintaining a Source Checkout](Maintaining-a-Source-Checkout).

## Troubleshooting

### Missing include with CLT 10.10.3

If you upgrade to CLT 10.10.3, you might get this error:

```
[100%] Building CXX object CMakeFiles/rmw_connext_cpp.dir/src/functions.cpp.o
In file included from /Users/william/ros2/src/ros2/rmw_connext/rmw_connext_cpp/src/functions.cpp:15:
In file included from /Library/Developer/CommandLineTools/usr/bin/../include/c++/v1/iostream:38:
In file included from /Library/Developer/CommandLineTools/usr/bin/../include/c++/v1/ios:216:
In file included from /Library/Developer/CommandLineTools/usr/bin/../include/c++/v1/__locale:15:
In file included from /Library/Developer/CommandLineTools/usr/bin/../include/c++/v1/string:439:
In file included from /Library/Developer/CommandLineTools/usr/bin/../include/c++/v1/algorithm:628:
In file included from /Library/Developer/CommandLineTools/usr/bin/../include/c++/v1/memory:604:
/Library/Developer/CommandLineTools/usr/bin/../include/c++/v1/iterator:341:10: fatal error: '__debug' file not found
#include <__debug>
         ^
1 error generated.
make[2]: *** [CMakeFiles/rmw_connext_cpp.dir/src/functions.cpp.o] Error 1
make[1]: *** [CMakeFiles/rmw_connext_cpp.dir/all] Error 2
make: *** [all] Error 2
```

I found this SO solution to work (http://stackoverflow.com/a/29576048/671658):

```
$ echo '#define _LIBCPP_ASSERT(x, m) ((void)0)' | sudo tee -a /Library/Developer/CommandLineTools/usr/include/c++/v1/__debug > /dev/null
```

It can be undone with:

```
$ sudo rm /Library/Developer/CommandLineTools/usr/include/c++/v1/__debug
```

### Missing symbol when opencv (and therefore libjpeg, libtiff, and libpng) are installed with Homebrew

If you have opencv installed you might get this:

```
dyld: Symbol not found: __cg_jpeg_resync_to_restart
  Referenced from: /System/Library/Frameworks/ImageIO.framework/Versions/A/ImageIO
  Expected in: /usr/local/lib/libJPEG.dylib
 in /System/Library/Frameworks/ImageIO.framework/Versions/A/ImageIO
/bin/sh: line 1: 25274 Trace/BPT trap: 5       /usr/local/bin/cmake
```

If so, to build you'll have to do this:

```
$ brew unlink libpng libtiff libjpeg
```

But this will break opencv, so you'll also need to update it to continue working:

```
$ sudo install_name_tool -change /usr/local/lib/libjpeg.8.dylib /usr/local/opt/jpeg/lib/libjpeg.8.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
$ sudo install_name_tool -change /usr/local/lib/libpng16.16.dylib /usr/local/opt/libpng/lib/libpng16.16.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
$ sudo install_name_tool -change /usr/local/lib/libtiff.5.dylib /usr/local/opt/libtiff/lib/libtiff.5.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
$ sudo install_name_tool -change /usr/local/lib/libjpeg.8.dylib /usr/local/opt/jpeg/lib/libjpeg.8.dylib /usr/local/Cellar/libtiff/4.0.4/lib/libtiff.5.dylib
```

The first command is necessary to avoid things built against the system libjpeg (etc.) from getting the version in /usr/local/lib.
The others are updating things built by Homebrew so that they can find the version of libjpeg (etc.) without having them in /usr/local/lib.
