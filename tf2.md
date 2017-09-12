# Using tf2 with ROS 2

There is preliminary support for [tf2](http://wiki.ros.org/tf2) in ROS 2. We rely heavily on tf2 in ROS 1 to manage data about coordinate transforms, and we expect to continue to use extensively in ROS 2.

Here's how to try it out. In each shell, be sure to start by sourcing the ROS 2 setup file as usual (e.g. on Linux `. ~/ros2_ws/install/setup.bash` or on Windows `call C:\dev\ros2\install\setup.bat`).

## Publishing transform data

First run the [static_transform_publisher](http://wiki.ros.org/tf2_ros#static_transform_publisher) to generate `tf2` data:

    ros2 run tf2_ros static_transform_publisher -- 1 2 3 0.5 0.1 -1.0 foo bar

That tool will publish a static transform from the parent frame `foo` to the child frame `bar` with (X, Y, Z) translation (1, 2, 3) and (roll, pitch, yaw) rotation (0.5, 0.1, -1.0).

## Receiving transform data

Now we can check whether it's possible to receive that transform data with `tf2_echo`:

    ros2 run tf2_ros tf2_echo -- foo bar

You should see repeated output similar to this:

~~~
At time 0.0
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [-0.475, -0.076, 0.240, 0.843]
~~~

Note that `tf2_echo` is reporting the rotation as a quaternion as opposed to roll, pitch, and yaw.
