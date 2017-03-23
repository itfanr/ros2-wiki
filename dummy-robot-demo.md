Within this demo, we present a simple demo robot with all components from publishing joint states over publishing fake laser data until visualizing the robot model on a map in RViz.

## Prerequisites
For the time of this writing, this demo requires a compilation from source as this demo was developed after the binary packages for beta1. Please make sure to refer to the (Install Instruction)[https://github.com/ros2/ros2/wiki/Installation] in order to verify your installation.

As a second prerequisite, we need the ROS1 bridge available. It is not strictly required, however without a functioning bridge, we won't have any chance to visualize the result of this demo in RViz.

## Launching the demo
We assume at this point that your ROS1 is setup correctly. We further assume your ROS2 installation dir as `~/ros2_ws`. Please change the directories according to your platform.

Terminal 1:
```
source /opt/ros/kinetic/setup.bash
roscore
``` 

Terminal 2:
```
source ~/ros2_ws/install/setup.bash
source /opt/ros/kinetic/setup.bash
dynamic_bridge --bridge-all-2to1-topics
```

Up to this point, we didn't launch anything special. We launched our ROS1 core and enabled to ROS1 bridge for forwarding all incoming ROS2 topics into the ROS1 ecosystem.

In the next step, we actually start the demo. We therefore execute the demo bringup launch file, which we are going to explain in more detail in the next section.

Terminal 3:
```
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/install/share/dummy_robot_bringup/launch/dummy_robot_bringup.py
```

You should see some prints inside your terminal:
```
python3 workspace/osrf/ros2_ws/install_isolated/dummy_robot_bringup/share/dummy_robot_bringup/launch/dummy_robot_bringup.py
(0) pid 52556: ['dummy_laser'] (stderr > stdout, all > console)
(1) pid 52557: ['dummy_map_server'] (stderr > stdout, all > console)
(2) pid 52558: ['robot_state_publisher', '/Users/karsten/workspace/osrf/ros2_ws/src/ros2/demos/dummy_robot/dummy_robot_bringup/launch/single_rrbot.urdf'] (stderr > stdout, all > console)
(3) pid 52559: ['dummy_joint_states'] (stderr > stdout, all > console)
[2] Initialize urdf model from file: /Users/karsten/workspace/osrf/ros2_ws/src/ros2/demos/dummy_robot/dummy_robot_bringup/launch/single_rrbot.urdf
[2] Parsing robot urdf xml string.
[2] Link single_rrbot_link1 had 1 children
[2] Link single_rrbot_link2 had 1 children
[2] Link single_rrbot_link3 had 2 children
[2] Link single_rrbot_camera_link had 0 children
[2] Link single_rrbot_hokuyo_link had 0 children
[2] got segment single_rrbot_camera_link
[2] got segment single_rrbot_hokuyo_link
[2] got segment single_rrbot_link1
[2] got segment single_rrbot_link2
[2] got segment single_rrbot_link3
[2] got segment world
[2] Adding fixed segment from world to single_rrbot_link1
[2] Adding moving segment from single_rrbot_link1 to single_rrbot_link2
[2] Adding moving segment from single_rrbot_link2 to single_rrbot_link3
[2] Adding fixed segment from single_rrbot_link3 to single_rrbot_camera_link
[2] Adding fixed segment from single_rrbot_link3 to single_rrbot_hokuyo_link
```

If you now open in a next terminal your RViz, you'll see your robot. 🎉 
Terminal 4:
```
source /opt/ros/kinetic/setup.bash
# upload the robot description for the robot model
rosparam set robot_description "`cat ~/ros2_ws/install/share/dummy_robot_bringup/launch/single_rrbot.urdf`"
rviz
```

After you configured your RViz accordingly, you should see a similar picture:
<fill in screenshot here>

### What's happening?
If you have a closer look at the launch file, we start a couple of nodes at the same time.

* dummy_map_server
* dummy_laser
* dummy_joint_states
* robot_state_publisher

The first two packages are relatively simple. The `dummy_map_server` constantly publishes an empty map with a period update. The `dummy_laser` does basically the same; publishing dummy fake laser scans.

The `dummy_joint_states` node is publishing fake joint state data. As we are publishing a simple RRbot with only two joints, this node publishes joint states values for these two joints.

The robot_state_publisher is doing the actual interesting work. It parses the given URDF file, extracts the robot model and listens to the incoming joint states. With this information, it publishes TF values for our robot which we then bridge and finally visualize in RViz.