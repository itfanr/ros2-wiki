# Installing ROS2 via Debian Packages

For Beta 2 we have started rolling Debian packages. They are in a temporary repository for testing.

Resources:
 - [Jenkins Instance](http://build.ros2.org/)
 - [Repositories](http://repo.ros2.org)
 - Status Pages ([amd64](http://repo.ros2.org/status_page/ros_r2b2_default.html), [arm64](http://repo.ros2.org/status_page/ros_r2b2_uxv8.html))

## Setup Sources

To install the Debian packages you will need to add our Debian repository to your apt sources.
First you will need to authorize our gpg key with apt like this:

```
sudo apt update && sudo apt install curl
curl http://repo.ros2.org/repos.key | sudo apt-key add -
```

And then add the repository to your sources list:

```
sudo echo "deb http://repo.ros2.org/ubuntu/main xenial main" > /etc/apt/sources.list.d/ros2-latest.list
```

## Install ROS 2 packages

The following commands install all `ros-r2b2-*` package except `ros-r2b2-ros1-bridge` and `ros-r2b2-turtlebot2-*` since they require ROS 1 dependencies.
See below for how to also install those.

```
sudo apt update
sudo apt install `apt list ros-r2b2-* 2> /dev/null | grep "/" | awk -F/ '{print $1}' | grep -v -e ros-r2b2-ros1-bridge -e ros-r2b2-turtlebot2- | tr "\n" " "`
```

## Environment setup

```
source /opt/ros/r2b2/setup.bash
```

If you have installed the Python package `argcomplete` you can source the following file to get completion for command line tools like `ros2`:

```
source /opt/ros/r2b2/share/ros2cli/environment/ros2-argcomplete.bash
```

## Additional packages using ROS 1 packages

The `ros1_bridge` as well as the TurtleBot demos are using ROS 1 packages.
To be able to install them please start by adding the ROS 1 sources as documented [here](http://wiki.ros.org/Installation/Ubuntu?distro=kinetic)

If you're using Docker for isolation you can start with the image `ros:kinetic` or `osrf/ros:kinetic-desktop`
This will also avoid the need to setup the ROS sources as they will already be integrated.

Now you can install the remaining packages:

```
sudo apt update
sudo apt install ros-r2b2-ros1-bridge ros-r2b2-turtlebot2-*
```
