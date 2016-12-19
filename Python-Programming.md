## Background

One of the most popular features of ROS 1 is the ability of writing robot applications in a variety of languages. Starting with Alpha 4, ROS 2 provides a Python API that can be used for communicating with other nodes written in different languages.

## Install

The `rclpy` module enables developers to use the ROS 2 API from Python. As of Alpha 4, `rclpy` is still under development and its API might change, but for now the basics (publishers and subscriptions) can be accessed from Python.

`rclpy` is part of the standard ROS 2 distribution, so it'll be installed alongside the rest of the ROS 2 packages.

## Run the examples

The two classic ROS 2 examples of a "talker" and a "listener" have been implemented in Python using `rclpy`. The source code is available [here](https://github.com/ros2/demos/tree/master/demo_nodes_py).

Open two terminals, source the appropriate `setup.*`/`local_setup.*` file in the terminals, and run `talker_py` in one and `listener_py` in the other.


**_Special case for users on Linux, with ROS 2 installed from pre-built binary packages, for ROS 2 releases up to and including alpha 8:_**

You'll need to invoke the examples with the Python executable directly: use ```python3 `which talker_py` ``` and ```python3 `which listener_py` ```


Once executed you should see the following on the terminal running the listener:

```
I heard: [Hello World: 1]
I heard: [Hello World: 2]
I heard: [Hello World: 3]
I heard: [Hello World: 4]
I heard: [Hello World: 5]
I heard: [Hello World: 6]
I heard: [Hello World: 7]
I heard: [Hello World: 8]
I heard: [Hello World: 9]
I heard: [Hello World: 10]
```

## Communication with nodes using different ROS client libraries
Since `rclpy` uses the underlying ROS 2 infrastructure, Python programs that use `rclpy` can also communicate with applications written in C++, even if they use a different DDS vendor and are running on a different operating system.