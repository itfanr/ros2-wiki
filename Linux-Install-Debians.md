# Installing ROS2 via Debian Packages

For Beta 2 we have started rolling debian packages. They are in a temporary repository for testing.

Resources: 
 - [Jenkins Instance](http://build.ros2.org/)
 - [Repositories](http://repo.ros2.org)
 - Status Pages ([amd64](http://repo.ros2.org/status_page/ros_r2b2_default.html), [arm64](http://repo.ros2.org/status_page/ros_r2b2_uxv8.html))

## ROS sources

Several of the demos reuse components from ROS1.
To be able to install them please start by adding the ROS 1 sources as documented [here](http://wiki.ros.org/Installation/Ubuntu?distro=kinetic)

If you're using Docker see the section with notes below.

## Setup Sources

To install from debians you will need to add our debian repository to your apt sources.
Do so like this:

```
echo "deb http://repo.ros2.org/ubuntu/testing xenial main" > /etc/apt/sources.list.d/ros2-latest.list
```

And then you will need to authorize our gpg key with apt like this:

```
curl http://repo.ros2.org/repos.key | apt-key add -
```

## Update

```
apt-get update
```

## Install 

```
apt-get install ros-r2b2-*
```

## Environment setup

```
source /opt/ros/r2b2/setup.bash
```

If you have installed the Python package `argcomplete` you can source the following file to get completion for command line tools like `ros2`:

```
source /opt/ros/r2b2/share/ros2cli/environment/ros2-argcomplete.bash
```

### Docker

Here are a few notes specific to Docker.
Docker can be a great environment for testing things out.

- If you're using Docker from a very stripped image like `ubuntu:xenial` you will need to install core dependencies like `lsb-release`.
  `apt-get update && apt-get install -y lsb-release curl
- If you're using Docker for isolation you can use the image `ros:kinetic`. 
Or if you'd like most of the ROS1 packages already installed for the demos `osrf/ros:kinetic-desktop`
This will also avoid the need to setup the ROS sources as they will already be integrated.