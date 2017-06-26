# Installing ROS2 via Debian Packages

For Beta 2 we have started rolling debian packages. They are in a temporary repository for testing.

## Setup Sources

```
echo "deb http://repo.ros2.org/ubuntu/testing $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list
```

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