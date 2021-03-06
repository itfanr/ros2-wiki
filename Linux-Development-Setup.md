# Building ROS 2 on Linux

## System Requirements

We support Ubuntu Linux Xenial Xerus 16.04 on 64-bit (until alpha 6 we supported Trusty Tahr 14.04).  These instructions should also work for later Ubuntu as well as Debian Stretch, though these are not actively tested or supported.  Fedora 26 also works if you follow [[alternate|Fedora-Development-Setup]] instructions, though it is not actively tested or supported. The same goes for [[Arch Linux|https://wiki.archlinux.org/index.php/Ros#Ros_2]].

Make sure that you have a locale set which supports `UTF-8` We test with the following settings.
If you are in a minimal environment such as a docker containers the locale may be set to something minimal like POSIX.
To set the locale an example is below. It should be fine if you're using a different UTF-8 supported locale.

```
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## How to setup the development environment?

First make sure you have the ROS apt repositories added to your system, if not:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Install GCC, G++ CMake, Python 3 EmPy package (custom packages which don't collide) and setuptools:

```
sudo apt-get update
sudo apt-get install git wget
sudo apt-get install build-essential cppcheck cmake libopencv-dev python-empy python3-dev python3-empy python3-nose python3-pip python3-pyparsing python3-setuptools python3-vcstool python3-yaml libtinyxml-dev libeigen3-dev
# dependencies for testing
sudo apt-get install clang-format pydocstyle pyflakes python3-coverage python3-mock python3-pep8 uncrustify
# Install argcomplete for command-line tab completion from the ROS2 tools.
# Install from pip rather than from apt-get because of a bug in the Ubuntu 16.04 version of argcomplete:
sudo pip3 install argcomplete
# additional testing dependencies from pip (because not available on ubuntu 16.04)
sudo pip3 install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest pytest-cov pytest-runner
# dependencies for FastRTPS
sudo apt-get install libasio-dev libtinyxml2-dev
```

If you are compiling **beta-1 or older** you also need to install boost for Fast-RTPS
```
sudo apt-get install libboost-chrono-dev libboost-date-time-dev libboost-program-options-dev libboost-regex-dev libboost-system-dev libboost-thread-dev

```
<!--
Keeping these because we can't remember if they were required for Jessie or for another reason. Are very likely not needed at all
```
# dependencies for turtlebot demo ?
sudo apt-get install libbz2-dev libreadline-dev libsqlite3-dev
```
-->
```
# dependencies for RViz
sudo apt-get install libcurl4-openssl-dev libqt5core5a libqt5gui5 libqt5opengl5 libqt5widgets5 libxaw7-dev libgles2-mesa-dev libglu1-mesa-dev qtbase5-dev
```

### Get ROS 2.0 code

Create a workspace and clone all repos:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs-import src < ros2.repos
```

> Note: if you want to get all of the latest bug fixes then you can try the "tip" of development by replacing `release-latest` in the URL above with `master`. The `release-latest` is preferred by default because it goes through more rigorous testing on release than changes to master do. See also [Maintaining a Source Checkout](https://github.com/ros2/ros2/wiki/Maintaining-a-Source-Checkout).

### Install one or more DDS implementations

ROS 2.0 builds on top of DDS.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.
The repositories you downloaded for ROS 2.0 includes eProsima's Fast RTPS, which is the only bundled vendor.
If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2.0 build will automatically build support for vendors that have been installed and sourced correctly.

By default we include eProsima's FastRTPS in the workspace and it is the default middleware. Detailed instructions for installing other DDS vendors are provided in the "Alternative DDS sources" section below.

As of Beta 3 using PrismTech OpenSplice is also supported again.
On Linux you can use a Debian package built by OSRF.


#### PrismTech OpenSplice Debian Packages built by OSRF


Set up the ROS 2 repository by following the [Setup Sources](https://github.com/ros2/ros2/wiki/Linux-Install-Debians#setup-sources) section of [this guide](https://github.com/ros2/ros2/wiki/Linux-Install-Debians#setup-sources).

```
sudo apt-get update
sudo apt-get install libopensplice67  # from repo.ros2.org
```

### Build the prototype using the bootstrap script from ament_tools

Note: to build the ROS 1 bridge, read the [ros1_bridge instructions](https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source).

More info on working with an ament workspace can be found in [this tutorial](Ament-Tutorial).

```
cd ~/ros2_ws/
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
```

Note: if you are having trouble compiling all examples and this is preventing you from completing a successful build, you can use `AMENT_IGNORE` in the same manner as [`CATKIN_IGNORE`](https://github.com/ros-infrastructure/rep/blob/master/rep-0128.rst) to ignore the subtree or remove the folder from the workspace.
Take for instance: you would like to avoid installing the large OpenCV library.
Well then simply `$ touch AMENT_IGNORE` in the `cam2image` demo directory to leave it out of the build process.

Optionally build all packages in isolation:

```
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --isolated
```

Afterwards source the `local_setup.*` from the `install` / `install_isolated` folder.

### Try some examples

In one terminal, source the setup file and then run a `talker`:
```
. ~/ros2_ws/install/local_setup.bash
# . ~/ros2_ws/install_isolated/local_setup.bash  # if you built in isolation, use this instead of the above line
ros2 run demo_nodes_cpp talker
```
In another terminal source the setup file and then run a `listener`:
```
. ~/ros2_ws/install/local_setup.bash
# . ~/ros2_ws/install_isolated/local_setup.bash  # if you built in isolation, use this instead of the above line
ros2 run demo_nodes_py listener
```
You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

See the [demos](Tutorials) for other things to try.

### Alternative DDS sources

The demos will attempt to build against any detected DDS vendor.
The only bundled vendor is eProsima's Fast RTPS, which is included in the default set of sources for ROS 2.0.
If you would like to switch out the vendor below are the instructions.
When you run the build make sure that your chosen DDS vendor(s) are exposed in your environment.

When multiple vendors are present, you can choose the used RMW implementation by setting the environment variable `RMW_IMPLEMENTATION` to the package providing the RMW implementation.
If the environment variable is not set the "default" vendor will be used (for now this is `rmw_fastrtps_cpp` if available, otherwise the first in alphabetical order).
For example, the `talker` binary will use the "default" vendor, but it can be invoked with different vendors, e.g. `RMW_IMPLEMENTATION=rmw_connext_cpp talker`.

#### In beta 1 and earlier

In the beta 1 and earlier releases the `RMW_IMPLEMENTATION` environment variable was not yet supported.
Instead multiple binaries were being provided.
For example, the `talker` binary will use the "default" vendor, but for each vendor there will also be a vendor specific binary for `talker`, e.g. `talker__rmw_connext_cpp`.
That way you can ensure you're using the right vendor for each binary by including the suffix when you run the program.

#### PrismTech OpenSplice (version 6.7.170912 or higher)

Choose one of the following options for PrismTech OpenSplice.

##### Debian packages built by OSRF (used in the default instruction sequence above)

```
sudo apt-get update
sudo apt-get install libopensplice67  # from repo.ros2.org
```

Add this to your `~/.bashrc`

```
export OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml
```

##### Official binary packages from PrismTech

Install the packages provided by [OpenSplice](https://github.com/PrismTech/opensplice/releases/tag/OSPL_V6_7_170912OSS_RELEASE) (we require at least version 6.7.170912).
Remember to replace `@@INSTALLDIR@@` with the path where you unpacked the OpenSplice distribution.
Then, source the ROS `setup.bash` file, and finally, source the `release.com` file in the root of the OpenSplice distribution to set the `OSPL_HOME` environment variable appropriately.
After that, your shell is ready to run ROS2 binaries with the official OpenSplice distribution.

You may also need to add the following line to your `.bashrc` file:

```
export PTECH_LICENSE_FILE=path/to/prismtech.lic
```

##### Building OpenSplice from source

If you build OpenSplice from source, be sure to remember to following the INSTALL.txt instructions and manually replace the @@INSTALLDIR@@ placeholder in the OpenSplice install/HDE/x86_64.linux/release.com

#### RTI Connext (version 5.3)

To use RTI Connext you will need to have obtained a license from RTI.
Add the following line to your `.bashrc` file pointing to your copy of the license.

```
export RTI_LICENSE_FILE=path/to/rti_license.dat
```

You also need a Java runtime installed to run the RTI code generator:

```
sudo apt-get install openjdk-7-jre
```

Finally, you can install the Connext 5.3 package for Linux provided by RTI from their [downloads page](https://www.rti.com/downloads).

After downloading, use `chmod +x` on the `.run` executable and then execute it.
Note that if you're installing to a system directory use `sudo` as well.

The default location is `~/rti_connext_dds-5.3.0`

Source the setup file to set the `NDDSHOME` environment variable.

```
source ~/rti_connext_dds-5.3.0/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash
```

Now you can build as normal and support for RTI will be built as well.

## Alternate compilers

Using a different compiler besides gcc to compile ROS 2 is easy. If you set the environment variables `CC` and `CXX` to executables for a working C and C++ compiler, respectively, and retrigger CMake configuration (by using `--force-cmake-config` or by deleting the packages you want to be affected), CMake will reconfigure and use the different compiler.

### Clang

To configure CMake to detect and use Clang:

```
sudo apt-get install clang
export CC=clang
export CXX=clang++
ament build --force-cmake-config
```

TODO: using ThreadSanitizer, MemorySanitizer

## Troubleshooting

### Internal compiler error

If you experience an ICE when trying to compile on a memory constrained platform like a Raspberry PI you might want to build single threaded (append `--make-flags -j1` to the `ament.py` invocation).

### Out of memory

The `ros1_bridge` in its current form requires 4Gb of free RAM to compile.
If you don't have that amount of RAM available it's suggested to use `AMENT_IGNORE` in that folder and skip its compilation.

### Multiple Host Interference

If you're running multiple instances on the same network you may get interference.
To avoid this you can set the environment variable `ROS_DOMAIN_ID` to a different integer, the default is zero.
This will define the DDS domain id for your system.
Note that if you are using the OpenSplice DDS implementation you will also need to update the OpenSplice configuration file accordingly. The location of the configuration file is referenced in the `OSPL_URI` environment variable.
