## Overview

This covers how to install ROS2 on linux from binary tarballs. The current target platform is Ubuntu Linux Trusty Tahr 14.04.


Install the OSRF sources for DDS via debian package:
```
apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list
```

Install runtime dependencies and wget:
```
apt-get update && apt-get install -q -y \
    libopencv-core2.4 \
    libhighgui2.4 \
    libopencv-imgproc2.4 \
    libopensplice64 \
    wget
```

Fetch the binaries and untar them:
```
mkdir -p ~/ros2_demo
cd ~/ros2_demo
wget -q https://github.com/ros2/ros2/releases/download/pre-alpha-rc1/ros2-package-linux.tar.bz2
tar -xf ros2-package-linux.tar.bz2
```

Enter the environment:
```
. ~/ros2_demo/ros2/local_setup.bash
```

Now you can run talker and listener, they are on your path!

See the [demos](Tutorials) for other things to try.