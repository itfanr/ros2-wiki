# Building ROS 2 on Linux

## System Requirements

We support Ubuntu Linux Trusty Tahr 14.04 on 64-bit.

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

Install GCC, G++ CMake, Python3 EmPy package (custom packages which don't collide) and setuptools:

```
sudo apt-get update
sudo apt-get install git wget
sudo apt-get install build-essential cppcheck cmake libopencv-dev python-empy python3-empy python3-setuptools python3-nose python3-pip python3-vcstool
# dependencies for fastrtps
sudo apt-get install libboost-chrono-dev libboost-date-time-dev libboost-regex-dev libboost-system-dev libboost-thread-dev
```

The setuptools version shipped with Ubuntu Trusty is not recent enough - we require at least version 8.2 (see https://bitbucket.org/pypa/setuptools/pull-request/85/):

```
sudo pip3 install -U setuptools
```

### Get ROS 2.0 code

Create a workspace and clone all repos:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs import ~/ros2_ws/src < ros2.repos
```

Note: In order to build the FastRTPS middleware option, edit the `ros2.repos` file and uncomment these two entries for `Fast-RTPS`, `Fast-CDR`,  and `ROS-RMW-Fast-RTPS-cpp`. Then rerun the `vcs import ...` command.

### Install one or more DDS implementations

ROS 2.0 builds on top of DDS.
It is compatible with multiple vendors.
You can choose to install one or more of the supported versions of DDS.

By default we will demonstrate installing PrismTech OpenSplice using Debian packages built by OSRF.
Detailed instructions for installing other DDS vendors are provided in the "Alternative DDS sources" section below.

#### PrismTech OpenSplice Debian Packages built by OSRF

```
sudo apt-get update
sudo apt-get install libopensplice64  # from packages.osrfoundation.org
```

### Build the prototype using the bootstrap script from ament_tools

Note: to build the ROS 1 bridge, read the [ros1_bridge instructions](https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source).

More info on working with an ament workspace can be found in [this tutorial](Ament-Tutorial).

```
cd ~/ros2_ws/
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
```

Note: if you are having trouble compiling all examples and this is preventing you from completing a successful build, you can usine `AMENT_IGNORE` is that same manner as [`CATKIN_IGNORE`](https://github.com/ros-infrastructure/rep/blob/master/rep-0128.rst) to ignore the subtree or remove the folder from the workspace.
Take for instance: you would like to avoid installing the large opencv library.
Well then simply `$ touch AMENT_IGNORE` file in the `cam2image` demo directory to leave it out of the build process.

Optionally build all packages in isolation:

```
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --isolated
```

Afterwards source the `local_setup.*` from the `install` / `install_isolated` folder.

Now that you have finished building the workspace, you can run talker and listener, they are on your path.
See the [demos](Tutorials) for other things to try.

### Alternative DDS sources

The demos will attempt to build against any detected DDS vendor.
If you would like to switch out the vendor below are the instructions.
When you run the build make sure that your chosen DDS vendor(s) are exposed in your environment.

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

#### RTI Connext

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

The default location is `/opt/rti_connext_dds-5.2.0`

Source the setup file to set the `NDDSHOME` environment variable.

```
source /opt/rti_connext_dds-5.2.0/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash
```

##### Debian packages built by OSRF

These packages are not public due to pending license questions.
Download the three Debian packages of the latest release from https://github.com/osrf/rticonnextdds-src/releases and install them using `dpkg -i`.

Note, if you have trouble `wget`'ing those `.deb` files, remember you have to be logged in because it is a private repo for now.

## Installation on different Linux than Ubuntu 14.04

### Debian Jessie/Stretch

In order to use the ROS 1 components with debian have look at http://wiki.ros.org/indigo/Installation/Debian. Otherwise you can also wait a few month until most ROS 1 components are ported to debian unstable (The core components are waiting to be unblocked by the ftp-masters at the moment). You don't need the ROS 1 componets in order to test ROS 2 without the ROS 1 Bridge.


Then add the osrf (gazebo) debian repository:

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'
sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
```

Install GCC, G++ CMake, Python3 EmPy package (custom packages which don't collide) and setuptools:

```
sudo apt-get update
sudo apt-get install git wget
sudo apt-get install build-essential cppcheck cmake libopencv-dev python-empy python3-empy python3-setuptools python3-nose python3-pip 
# dependencies for fastrtps
sudo apt-get install libboost-chrono-dev libboost-date-time-dev libboost-regex-dev libboost-system-dev libboost-thread-dev
```

Then run:
```
sudo pip3 install vcstool
```

Afterwards you can proceed with [Get ROS 2.0 code](#get-ros-20-code).

## Troubleshooting

### Out of memory

The ros1_bridge in its current form requires 4Gb of free RAM to compile.
If you don't have that amount of RAM available it's suggested to use `AMENT_IGNORE` in that folder and skip it's compilation.

### Multiple Host Interference

If you're running multiple instances on the same network you may get interference.
To avoid this you can set the environment variable `ROS_DOMAIN_ID` to a different integer, the default is zero.
This will define the DDS domain id for your system.
