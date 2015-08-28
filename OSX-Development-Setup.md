# Installing ROS 2 from source on OS X

## System requirements
We support OS X Yosemite (10.10.x).

## Install prerequisites
**TODO: Extend these instructions for other DDS implementations, starting with RTI Connext.**

You need the following things installed to build ROS 2:

 1. **Java Development Kit (JDK)** *(currently required to compile the OpenSplice DDS implementation; that requirement might go away in the future, e.g., if we disable building their Java bindings)*:
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

        # We're disabling python support in opencv to avoid a dependency on numpy,
        # which in turn will want to build gcc 5.2.0, which takes a long time.
        brew install opencv --without-python
        brew install python3 wget cmake cppcheck opensplice gtest
    *Optional*: Make sure that you have `gtest` 1.7:

        $ brew info gtest
        ros/deps/gtest: stable 1.7.0
        http://code.google.com/p/googletest/
        /usr/local/Cellar/gtest/1.7.0 (32 files, 1.2M) *
          Built from source
        From: https://github.com/ros/homebrew-deps/blob/master/gtest.rb
 1. Use `pip3` (**not `pip`**, which will install Python 2.7 packages!) to install more stuff:

        pip3 install empy setuptools nose vcstool
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
    wget https://raw.githubusercontent.com/ros2/examples/master/ros2.repos
    vcs import ~/ros2_ws/src < ros2.repos

## Build the ROS 2 code
**Note**: if you installed and sourced the setup file for ROS 1, then you should instead follow the [modified instructions](#building-with-the-ros-1-bridge).

Run the `ament` tool to build everything (more on using `ament` in [[this tutorial|Ament-Tutorial]]):

    cd ~/ros2_ws/
    src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install

### Building with the ROS 1 bridge
The ROS 1 bridge requires a patched version of `rosbag` (for Python 3 compatibility) that has not yet been released.
As a quick workaround, we're going to comment out one line in `rosbag`, build the ROS 1 bridge, then revert that change (to make your ROS 1 `rosbag` package work again).

Also, building the ROS 1 bridge can consume a tremendous amount of memory to the point that it can easily overwhelm a computer if done with parallel compilation enabled.
As such, we recommend first building everything else as usual, then coming back to build the ROS 1 bridge without parallel compilation.

Here are the steps:

    # Patch rosbag to remove non-Python3-compatible line
    sed -i '' 's/import roslz4/#import roslz4/' $ROS_ROOT/../../lib/python2.7/site-packages/rosbag/bag.py
    # Ignore ros1_bridge and build everything else
    touch src/ros2/ros1_bridge/AMENT_IGNORE
    src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
    # Un-ignore ros1_bridge and build it, non-parallel
    rm src/ros2/ros1_bridge/AMENT_IGNORE
    src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --only ros1_bridge --make-flags -j1
    # Un-patch rosbag to put back the non-Python3-compatible line
    sed -i '' 's/#import roslz4/import roslz4/' $ROS_ROOT/../../lib/python2.7/site-packages/rosbag/bag.py

## Try some examples
In one terminal, source the setup file and then run a `talker`:

    . ~/ros2_ws/install/setup.bash
    talker
In another terminal source the setup file and then run a `listener`:

    . ~/ros2_ws/install/setup.bash
    listener
You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

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
$ sudo install_name_tool -change /usr/local/lib/libpng16.16.dylib /usr/local/Cellar/libpng/1.6.17/lib/libpng16.16.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
$ sudo install_name_tool -change /usr/local/lib/libtiff.5.dylib /usr/local/Cellar/libtiff/4.0.4/lib/libtiff.5.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
$ sudo install_name_tool -change /usr/local/lib/libjpeg.8.dylib /usr/local/opt/jpeg/lib/libjpeg.8.dylib /usr/local/Cellar/libtiff/4.0.4/lib/libtiff.5.dylib
```

The first command is necessary to avoid things built against the system libjpeg (etc.) from getting the version in /usr/local/lib.
The others are updating things built by Homebrew so that they can find the version of libjpeg (etc.) without having them in /usr/local/lib.

## Maintainer notes
To build a binary package for distribution, follow the steps described above, then:

    mv install ros2
    tar cvfL ros2-package-osx.tar ros2
    bzip2 ros2-package-osx.tar
Ship it!