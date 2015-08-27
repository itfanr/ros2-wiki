For OSX see: https://github.com/ros2/examples/wiki/OSX-Development-Setup
For Windows see: https://github.com/ros2/examples/wiki/Windows-Development-Setup

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
wget https://raw.githubusercontent.com/ros2/examples/master/ros2.repos
vcs import ~/ros2_ws/src < ros2.repos
```

### Install one or more DDS implementations

ROS 2.0 builds on top of DDS.
It is compatible with multiple vendors.
You can choose to install one or more of the supported versions of DDS.

By default we will demonstrate installing opensplice from OSRF packaged debian packages.
Details for installing other DDS vendors is below.

#### PrismTech OpenSplice Debian Packages built by OSRF

```
sudo apt-get update
sudo apt-get install libopensplice64  # from packages.osrfoundation.org
```


### Build the prototype using the bootstrap script from ament_tools

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

Now that you have finished building the workspace, try out the demos.
TODO LINKME

## Alternative DDS souces.

#### PrismTech OpenSplice

If you are using a

Add the following line to you `.bashrc` file:
```
export PTECH_LICENSE_FILE=path/to/prismtech.lic
```

Choose one of the following options for PrismTech OpenSplice.

##### Debian packages built by OSRF

```
sudo apt-get update
sudo apt-get install libopensplice64  # from packages.osrfoundation.org
```

Add this to you `~/.bashrc`
```
export OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml
```

##### Official binary packages from PrismTech

Install the packages provided by [OpenSplice](http://www.prismtech.com/dds-community/software-downloads) (we currently use 6.4.1p2).
Source the `release.com` file to set the `OSPL_HOME` environment variable.

##### Building OpenSplice from source

If you build OpenSplice from source, be sure to remember to following the INSTALL.txt instructions and manually replace the @@INSTALLDIR@@ placeholder in the OpenSplice install/HDE/x86_64.linux/release.com

#### RTI Connext

Add the following line to your `.bashrc` file:
```
export RTI_LICENSE_FILE=path/to/rti_license.dat
```

You also need a Java runtime installed to run the RTI code generator:

```
sudo apt-get install openjdk-7-jre
```

Choose one of the following options for RTI Connext.

##### Debian packages built by OSRF

These packages are not public due to pending license questions.
Download the three Debian packages of the latest release from https://github.com/osrf/rticonnextdds-src/releases and install them using `dpkg -i`.

Note, if you have trouble `wget`'ing those `.deb` files, remember you have to be logged in because it is a private repo for now.

Also add to your .bashrc `export NDDSHOME=/usr`

##### Official binary packages from RTI

Install the packages provided by [RTI](http://www.rti.com/downloads/connext-files.html#DOWNLOAD) - the Ubuntu 12.04 packages seem to work for us with Ubuntu 14.04.
Source the `rti_set_bash_5.1.0` file to set the `NDDSHOME` environment variable.


## Integration with Sublime Text 3

You might want to use the sublime package to integrate with the ament build system: https://github.com/ament/sublime-ament

## Troubleshooting

### Out of memory

The ros1_bridge in it's current form requires 4Gb of free RAM to compile.
If you don't have that amount of RAM available it's suggested to use `AMENT_IGNORE` in that folder and skip it's compilation.

### Multiple Host Interference

If you're running multiple instances on the same network you may get interference.
To avoid this you can set the environment variable `ROS_DOMAIN_ID` to a different integer, the default is zero.
This will define the DDS domain id for your system.

#### On OpenSplice

If you are using OpenSplice you will need to change the config file.
It should be accessible via `OSPL_URI` in your environment.
You can either edit the file in place or make a copy and update your environment variable to point to the new copy.

replace:
```
      <Id>0</Id>
```
with:
```
      <Id>${ROS_DOMAIN_ID}</Id>
```
In the debian packages this file is /usr/etc/opensplice/config/ospl.xml by default.

To do this quickly use the following
```
cp /usr/etc/opensplice/config/ospl.xml ~
sed -i 's|<Id>0</Id>|<Id>${ROS_DOMAIN_ID}</Id>|' ~/ospl.xml
echo export OSPL_URI=file://$HOME/ospl.xml >> ~/.bashrc
```