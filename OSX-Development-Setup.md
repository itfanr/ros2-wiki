# Building ROS 2 on OS X

## System requirements

We support OS X 10.12.x.

However, some older versions like 10.11.x and 10.10.x are known to work as well.

## Install prerequisites

You need the following things installed to build ROS 2:

1. **Xcode**
    * If you don't already have it installed, install Xcode and the Command Line Tools:
    ```
    xcode-select --install
    ```

1. **brew** *(needed to install more stuff; you probably already have this)*:
    * Follow installation instructions at http://brew.sh/
    * *Optional*: Check that `brew` is happy with your system configuration by running:

    ```
    brew doctor
    ```

    Fix any problems that it identifies.

1. Use `brew` to install more stuff:

        brew install python3 wget cmake cppcheck tinyxml eigen pcre

        # install dependencies for Fast-RTPS if you are using it
        brew install asio tinyxml2

        brew install opencv

1. Use `python3 -m pip` (just `pip` may install Python3 or Python2) to install more stuff:

        python3 -m pip install argcomplete coverage empy flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mock nose pep8 pydocstyle pyflakes pytest ptest-cov pytest-runner pyyaml setuptools vcstool

1. *Optional*: if you want to build the ROS 1<->2 bridge, then you must also install ROS 1:

    * Start with the normal install instructions: http://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source
    * When you get to the step where you call `rosinstall_generator` to get the source code, here's an alternate invocation that brings in just the minimum required to produce a useful bridge:

            rosinstall_generator catkin common_msgs roscpp rosmsg --rosdistro kinetic --deps --wet-only --tar > kinetic-ros2-bridge-deps.rosinstall
            wstool init -j8 src kinetic-ros2-bridge-deps.rosinstall

    Otherwise, just follow the normal instructions, then source the resulting `install_isolated/setup.bash` before proceeding here to build ROS 2.

## Disable System Integrity Protection (SIP)
OS X versions >=10.11 have System Integrity Protection enabled by default.
So that SIP doesn't prevent processes from inheriting dynamic linker environment variables, such as `DYLD_LIBRARY_PATH`, you'll need to disable it [following these instructions](https://developer.apple.com/library/content/documentation/Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html).

## Get the ROS 2 code

Create a workspace and clone all repos:

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
    vcs import src < ros2.repos

> Note: if you want to get all of the latest bug fixes then you can try the "tip" of development by replacing `release-latest` in the url above with `master`. The `release-latest` is preferred by default because it goes through more rigorous testing on release than changes to master do. See also [Maintaining a Source Checkout](https://github.com/ros2/ros2/wiki/Maintaining-a-Source-Checkout).

## Optional: Install additional DDS vendors

ROS 2.0 builds on top of DDS.
It is compatible with [[multiple DDS or RTPS (the DDS wire protocol) vendors|DDS-and-ROS-middleware-implementations]].
The repositories you downloaded for ROS 2.0 includes eProsima's Fast RTPS, which is the only bundled vendor.
If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2.0 build will automatically build support for vendors that have been installed and sourced correctly.

By default we include eProsima's FastRTPS in the workspace and it is the default middleware.
Detailed instructions for installing other DDS vendors are provided in the "Alternative DDS sources" section below.

## Build the ROS 2 code

**Note**: if you are trying to build the ROS 1 <-> ROS 2 bridge, follow instead these [modified instructions](https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source).

Run the `ament` tool to build everything (more on using `ament` in [[this tutorial|Ament-Tutorial]]):

    cd ~/ros2_ws/
    src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install

## Try some examples

In one terminal, source the setup file and then run a `talker`:

    . ~/ros2_ws/install/setup.bash
    ros2 run demo_nodes_cpp talker

In another terminal source the setup file and then run a `listener`:

    . ~/ros2_ws/install/setup.bash
    ros2 run demo_nodes_cpp listener

You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

## Alternative DDS sources

The demos will attempt to build against any detected DDS vendor.
The only bundled vendor is eProsima's Fast RTPS, which is included in the default set of sources for ROS 2.0.
If you would like to switch out the vendor below are the instructions.
When you run the build make sure that your chosen DDS vendor(s) are exposed in your environment.

When multiple vendors are present, you can choose the used RMW implementation by setting the the environment variable `RMW_IMPLEMENTATION` to the package providing the RMW implementation.
See [[Working with multiple RMW implementations|Working-with-multiple-RMW-implementations]] for more details.

<!-- commenting because opensplice 4.7 has not been released for osx
### PrismTech OpenSplice

To build opensplice you will need:

 1. **Java Development Kit (JDK)** *(currently required to compile the OpenSplice DDS implementation, but that requirement might go away in the future, e.g., if we disable building their Java bindings)*:
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

 1. Add the OSRF Homebrew tap:

        brew tap osrf/ros2
 1. Install OpenSplice:

        brew install opensplice
-->
### RTI Connext (5.2.3)

To use RTI Connext you will need to have obtained a license from RTI.

You can install the OS X package of Connext version 5.2.3 provided by RTI [from this link](http://s3.amazonaws.com/RTI/Bundles/5.2.3/Evaluation/rti_connext_dds-5.2.3-eval-x64Darwin15clang7.0.dmg).

You also need a Java runtime installed to run the RTI code generator, which you can get [here](https://support.apple.com/kb/DL1572?locale=en_US).

After installing, run RTI launcher and point it to your license file.

Source the setup file to set the `NDDSHOME` environment variable before building your workspace:

```
source /Applications/rti_connext_dds-5.2.3/resource/scripts/rtisetenv_x64Darwin15clang7.0.bash
```

You may need to increase shared memory resources following https://community.rti.com/kb/osx510.

## Troubleshooting

### Library not loaded; image not found

If you are seeing library loading issues at runtime (either running tests or running nodes), such as the following:

```
ImportError: dlopen(.../ros2_install/ros2-osx/lib/python3.6/site-packages/rclpy/_rclpy.cpython-36m-darwin.so, 2): Library not loaded: @rpath/librcl_interfaces__rosidl_typesupport_c.dylib
  Referenced from: .../ros2_install/ros2-osx/lib/python3.6/site-packages/rclpy/_rclpy.cpython-36m-darwin.so
  Reason: image not found
```

then you probably have System Integrity Protection enabled.
See "Disable System Integrity Protection (SIP)" above for how instructions on how to disable it.

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
