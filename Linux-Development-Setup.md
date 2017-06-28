# Building ROS 2 on Linux

## System Requirements

We support Ubuntu Linux Xenial Xerus 16.04 on 64-bit (until alpha 6 we supported Trusty Tahr 14.04). While not being actively tested these instructions should also work for later Ubuntu as well as Debian Stretch.

Make sure that you have a locale set which supports `UTF-8` We test with the following settings.
If you are in a minimal environment such as a docker containers the locale may be set to something minimal like POSIX.
To set the locale an example is below. It should be fine if you're using a different UTF-8 supported locale.

```
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

## How to setup the development environment?

First make sure you have the ROS apt repositories added to your system, if not:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Also get the osrf (gazebo) debian repository:

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
```

Install GCC, G++ CMake, Python 3 EmPy package (custom packages which don't collide) and setuptools:

```
sudo apt-get update
sudo apt-get install git wget
sudo apt-get install build-essential cppcheck cmake libopencv-dev libpoco-dev libpocofoundation9v5 libpocofoundation9v5-dbg python-empy python3-dev python3-empy python3-nose python3-pip python3-setuptools python3-vcstool python3-yaml libtinyxml-dev libeigen3-dev
# dependencies for testing
sudo apt-get install clang-format pydocstyle pyflakes python3-coverage python3-mock python3-pep8 uncrustify
sudo pip3 install flake8 flake8-import-order argcomplete
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
### Get ROS 2.0 code

Create a workspace and clone all repos:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs-import src < ros2.repos
```
This will get the code for the latest ROS 2 release. If you want the code from a particular release or from the development branches, see [this page](Maintaining-a-Source-Checkout).

### Install one or more DDS implementations

ROS 2.0 builds on top of DDS.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.
The repositories you downloaded for ROS 2.0 includes eProsima's Fast RTPS, which is the only bundled vendor.
If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2.0 build will automatically build support for vendors that have been installed and sourced correctly.

By default we include eProsima's FastRTPS in the workspace and it is the default middleware. Detailed instructions for installing other DDS vendors are provided in the "Alternative DDS sources" section below.

<!-- commenting out opensplice as we're not currently supporting it
By default we will demonstrate installing PrismTech OpenSplice using Debian packages built by OSRF.


#### PrismTech OpenSplice Debian Packages built by OSRF

```
sudo apt-get update
sudo apt-get install libopensplice64  # from packages.osrfoundation.org
```

-->

### Build the prototype using the bootstrap script from ament_tools

Note: to build the ROS 1 bridge, read the [ros1_bridge instructions](https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source).

More info on working with an ament workspace can be found in [this tutorial](Ament-Tutorial).

```
cd ~/ros2_ws/
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
```

Note: if you are having trouble compiling all examples and this is preventing you from completing a successful build, you can use `AMENT_IGNORE` in the same manner as [`CATKIN_IGNORE`](https://github.com/ros-infrastructure/rep/blob/master/rep-0128.rst) to ignore the subtree or remove the folder from the workspace.
Take for instance: you would like to avoid installing the large opencv library.
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
talker
```
In another terminal source the setup file and then run a `listener`:
```
. ~/ros2_ws/install/local_setup.bash
# . ~/ros2_ws/install_isolated/local_setup.bash  # if you built in isolation, use this instead of the above line
listener
```
You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!

See the [demos](Tutorials) for other things to try.

### Maintain your Source Checkout
For information on how to keep your source checkout up-to-date, see [Maintaining a Source Checkout](Maintaining-a-Source-Checkout).

### Alternative DDS sources

The demos will attempt to build against any detected DDS vendor.
The only bundled vendor is eProsima's Fast RTPS, which is included in the default set of sources for ROS 2.0.
If you would like to switch out the vendor below are the instructions.
When you run the build make sure that your chosen DDS vendor(s) are exposed in your environment.

When multiple vendors are present, you can choose the used RMW implementation by setting the the environment variable `RMW_IMPLEMENTATION` to the package providing the RMW implementation.
If the environment variable is not set the "default" vendor will be used (for now this is `rmw_fastrtps_cpp` if available, otherwise the first in alphabetical order).
For example, the `talker` binary will use the "default" vendor, but it can be invoked with different vendors, e.g. `RMW_IMPLEMENTATION=rmw_connext_cpp talker`.

#### In beta 1 and earlier

In the beta 1 and earlier releases the `RMW_IMPLEMENTATION` environment variable was not yet supported.
Instead multiple binaries were being provided.
For example, the `talker` binary will use the "default" vendor, but for each vendor there will also be a vendor specific binary for `talker`, e.g. `talker__rmw_connext_cpp`.
That way you can ensure you're using the right vendor for each binary by including the suffix when you run the program.

<!-- opensplice not currently supported hiding the instructions.
#### PrismTech OpenSplice

Choose one of the following options for PrismTech OpenSplice.

##### Debian packages built by OSRF (used in the default instruction sequence above)

```
sudo apt-get update
sudo apt-get install libopensplice64  # from packages.osrfoundation.org
```

Add this to your `~/.bashrc`

```
export OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml
```

##### Official binary packages from PrismTech

Install the packages provided by [OpenSplice](http://www.prismtech.com/dds-community/software-downloads) (we currently use 6.4.1p2).
Remember to replace `@@INSTALLDIR@@` with the path where you unpacked the OpenSplice distribution.
Then, source the ROS `setup.bash` file, and finally, source the `release.com` file in the root of the OpenSplice distribution to set the `OSPL_HOME` environment variable appropriately.
After that, your shell is ready to run ROS2 binaries with the official OpenSplice distribution.

You may also need to add the following line to you `.bashrc` file:

```
export PTECH_LICENSE_FILE=path/to/prismtech.lic
```

##### Building OpenSplice from source

If you build OpenSplice from source, be sure to remember to following the INSTALL.txt instructions and manually replace the @@INSTALLDIR@@ placeholder in the OpenSplice install/HDE/x86_64.linux/release.com
-->

#### RTI Connext (version 5.2 or higher)

To use RTI Connext you will need to have obtained a license from RTI.
Add the following line to your `.bashrc` file pointing to your copy of the license.

```
export RTI_LICENSE_FILE=path/to/rti_license.dat
```

You also need a Java runtime installed to run the RTI code generator:

```
sudo apt-get install openjdk-7-jre
```

Choose one of the following options for RTI Connext.

##### Official binary packages from RTI

You can install the packages provided by [RTI](http://www.rti.com/downloads/connext-files.html#DOWNLOAD)  - Get the 14.04 version.

After downloading `chmod +x` on the .run the executable and execute.
(If you're installing to a system directory use `sudo`.)

The default location is `~/rti_connext_dds-5.2.3`

Source the setup file to set the `NDDSHOME` environment variable.

```
source ~/rti_connext_dds-5.2.3/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash
```

##### Debian packages built by OSRF

These packages are not public due to pending license questions.
If you have been granted access, you can download the three Debian packages of the latest release from https://github.com/osrf/rticonnextdds-src/releases and install them using `dpkg -i`.

Note, if you have trouble `wget`'ing those `.deb` files, remember you have to be logged in because it is a private repo for now.

## Alternate compilers

Using a different compiler besides gcc to compile ROS 2 is easy. If you set the environment variables `CC` and `CXX` to executables for a working C and C++ compiler, respectively, and retrigger CMake configuration (by using `--force-cmake-config` or by deleting the packages you want affected), CMake will reconfigure and use the different compiler.

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
If you don't have that amount of RAM available it's suggested to use `AMENT_IGNORE` in that folder and skip it's compilation.

### Multiple Host Interference

If you're running multiple instances on the same network you may get interference.
To avoid this you can set the environment variable `ROS_DOMAIN_ID` to a different integer, the default is zero.
This will define the DDS domain id for your system.
Note that if you are using the OpenSplice DDS implementation you will also need to update the OpenSplice configuration file accordingly. The location of the configuration file is referenced in the `OSPL_URI` environment variable.