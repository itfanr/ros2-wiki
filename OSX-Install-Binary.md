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

## Downloading ROS 2
* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for OS X; let's assume that it ends up at `~/Downloads/ros2-package-osx.tar.bz2`.
* Unpack it:

        mkdir -p ~/ros2_install
        cd ~/ros2_install
        tar xf ~/Downloads/ros2-package-osx.tar.bz2

## Try some examples
In one terminal, source the setup file and then run a `talker`:

    . ~/ros2_install/ros2/setup.bash
    talker
In another terminal source the setup file and then run a `listener`:

    . ~/ros2_install/ros2/setup.bash
    listener
You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

### ROS 1 bridge
If you have ROS 1 installed, you can try the ROS 1 bridge, by first sourcing your ROS 1 setup file (assume that it's `~/ros_catkin_ws/install_isolated/setup.bash`):

    . ~/ros_catkin_ws/install_isolated/setup.bash
    # If you haven't already, start a roscore:
    roscore &
    . ~/ros2_install/ros2/setup.bash
    dynamic_bridge
For more information on the bridge, read the [tutorial](https://github.com/ros2/ros1_bridge/blob/master/README.md).