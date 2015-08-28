# Installing ROS 2 on OS X
This page explains how to install ROS 2 on OS X from a pre-built binary package.

## Installing prerequisites
You need the following things installed before installing ROS 2.

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

        # OpenCV isn't a dependency of ROS 2, but it used by some demos.
        # We're disabling python support in opencv to avoid a dependency on numpy,
        # which in turn will want to build gcc 5.2.0, which takes a long time.
        brew install opencv --without-python
        brew install python3 opensplice
1. *Optional*: if you want to use the ROS 1<->2 bridge, then you must also install ROS 1.  Follow the normal install instructions: http://wiki.ros.org/indigo/Installation/OSX/Homebrew/Source

## 