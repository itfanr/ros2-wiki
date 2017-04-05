# Rosbag with ROS1 Bridge

This tutorial is a follow up to the "Bridge communication between ROS 1 and ROS 2" tutorial linked to from the [Tutorials page on this wiki](Tutorials), and this tutorial assumes you have completed that tutorial already.

**Note**: Be sure to follow the [instructions](https://github.com/ros2/ros1_bridge/blob/master/README.md#building-the-bridge-from-source) for building the bridge from source, which include building the `ros1_bridge` package on its own after building everything else.
Otherwise you'll end up without all the message support that you need.

What follows is a series of additional examples, like that ones that come at the end of the aforementioned "Bridge communication between ROS 1 and ROS 2" tutorial.

## Recording Topic Data with rosbag and ROS 1 Bridge

In this example, we'll be using the `cam2image` demo program that comes with ROS 2 and a Python script to emulate a simple turtlebot-like robot's sensor data so that we can bridge it to ROS 1 and use rosbag to record it.

First we'll run a ROS 1 `roscore` in a new shell:

```
# Shell A:
. /opt/ros/kinetic/setup.bash
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
roscore
```

Then we'll run the ROS 1 <=> ROS 2 `dynamic_bridge` with the `--bridge-all-topics` option (so we can do `rostopic list` and see them) in another shell:

```
# Shell B:
. /opt/ros/kinetic/setup.bash
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
. <workspace-with-bridge>/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
dynamic_bridge --bridge-all-topics
```

Remember to replace `<workspace-with-bridge>` with the path to where you either extracted the ROS 2 binary or where you built ROS 2 from source.

---

TODO: replace the `cam2image` and `emulate_kobuki_node.py` with actual astra camera and turtlebot when they're ready.

Now we can start up the ROS 2 programs that will emulate our turtlebot-like robot.
First we'll run the `cam2image` program with the `-b` option so it doesn't require a camera to work:

```
# Shell C:
. <workspace-with-bridge>/install/setup.bash
cam2image -b
```

TODO: use namespaced topic names

Then we'll run a simple Python script to emulate the `odom` and `imu_data` topics from a Kobuki base.
I would use the more accurate `~sensors/imu_data` topic name for the imu data, but we don't have namespace support just yet in ROS 2 (it's coming!).
Place this script in a file called `emulate_kobuki_node.py`:

```python
#!/usr/bin/env python3

import sys
import time

import rclpy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node('emulate_kobuki_node')

    imu_publisher = node.create_publisher(Imu, 'imu_data')
    odom_publisher = node.create_publisher(Odometry, 'odom')

    imu_msg = Imu()
    odom_msg = Odometry()
    counter = 0
    while True:
        counter += 1
        now = time.time()
        if (counter % 50) == 0:
            odom_msg.header.stamp.sec = int(now)
            odom_msg.header.stamp.nanosec = int(now * 1e9) % 1000000000
            odom_publisher.publish(odom_msg)
        if (counter % 100) == 0:
            imu_msg.header.stamp.sec = int(now)
            imu_msg.header.stamp.nanosec = int(now * 1e9) % 1000000000
            imu_publisher.publish(imu_msg)
            counter = 0
        time.sleep(0.001)


if __name__ == '__main__':
    sys.exit(main())

```

You can run this python script in a new ROS 2 shell:

```
# Shell D:
. <workspace-with-bridge>/install/setup.bash
python3 emulate_kobuki_node.py
```

---

Now that all the data sources and the dynamic bridge are running, we can look at the available topics in a new ROS 1 shell:

```
# Shell E:
. /opt/ros/kinetic/setup.bash
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
rostopic list
```

You should see something like this:

```
% rostopic list
/image
/imu_data
/odom
/rosout
/rosout_agg
```

We can now record this data with `rosbag record` in the same shell:

```
# Shell E:
rosbag record /image /imu_data /odom
```

After a few seconds you can `ctrl-c` the `rosbag` command and do an `ls -lh` to see how big the file is, you might see something like this:

```
% ls -lh
total 0
-rw-rw-r-- 1 william william  12M Feb 23 16:59 2017-02-23-16-59-47.bag
```

Though the file name will be different for your bag (since it is derived from the date and time).

## Playing Back Topic Data with rosbag and ROS 1 Bridge

Now that we have a bag file you can use any of the ROS 1 tools to introspect the bag file, like `rosbag info <bag file>`, `rostopic list -b <bag file>`, or `rqt_bag <bag file>`.
However, we can also playback bag data into ROS 2 using `rosbag play` and the ROS 1 <=> ROS 2 `dynamic_bridge`.

First closed out all the shells you opened for the previous tutorial, stopping any running programs.

Then in a new shell start the `roscore`:

```
# Shell P:
. /opt/ros/kinetic/setup.bash
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
roscore
```

Then run the `dynamic_bridge` in another shell:

```
# Shell Q:
so
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
. <workspace-with-bridge>/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
dynamic_bridge --bridge-all-topics
```

Then play the bag data back with `rosbag play` in another new shell, using the `--loop` option so that we don't have to keep restarting it for short bags:

```
# Shell R:
. /opt/ros/kinetic/setup.bash
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
rosbag play --loop path/to/bag_file
```

Make sure to replace `path/to/bag_file` with the path to the bag file you want to playback.

---

Now that the data is being played back and the bridge is running we can see the data coming across in ROS 2.

TODO: use rostopic like tools here

Since we don't have command line tools in ROS 2 yet (it's coming!) I used this Python script to see the data coming from the bag file (I used the file name `emulated_kobuki_monitor.py`):

```python
#!/usr/bin/env python3

import sys

import rclpy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu

odom_counter = 0
image_counter = 0
imu_counter = 0


def odom_callback(msg):
    global odom_counter
    odom_counter += 1
    if odom_counter % 50 == 0:
      print('odom messages received: %s' % odom_counter)


def image_callback(msg):
    global image_counter
    image_counter += 1
    if image_counter % 30 == 0:
      print('image messages received: %s' % image_counter)


def imu_callback(msg):
    global imu_counter
    imu_counter += 1
    if imu_counter % 100 == 0:
      print('imu messages received: %s' % imu_counter)


def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node('emulated_kobuki_monitor')

    odom_sub = node.create_subscription(Odometry, 'odom', odom_callback)
    image_sub = node.create_subscription(Image, 'image', image_callback)
    imu_sub = node.create_subscription(Imu, 'imu_data', imu_callback)

    while rclpy.ok():
        rclpy.spin_once(node)


if __name__ == '__main__':
    main()

```

Then you can run this in a new ROS 2 shell:

```
# Shell S:
. <workspace-with-bridge>/install/setup.bash
python3 ./emulated_kobuki_monitor.py
```

You might see something like this:

```
% python3 ./emulated_kobuki_monitor.py
odom messages received: 50
odom messages received: 100
imu messages received: 100
odom messages received: 150
odom messages received: 200
imu messages received: 200
image messages received: 30
...
```
