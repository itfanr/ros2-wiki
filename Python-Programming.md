## Background

One of the most popular features of ROS 1 is the ability of writing robot applications in a variety of languages. Starting with Alpha 4, ROS 2 provides a Python API that can be used for communicating with other nodes written in different languages.

## Install

The `rclpy` module enables developers to use the ROS 2 API from Python. As of Alpha 4, `rclpy` is still under development and its API might change, but for now the basics (publishers and subscriptions) can be accessed from Python.

`rclpy` is part of the standard ROS 2 distribution, so it'll be installed alongside the rest of the ROS 2 packages.

## Run the examples

Before running either you must source the appropriate `setup.*` file in your terminal.

The two classic ROS 2 examples have been implemented in Python using `rclpy`, open two terminals and run `talker_py` on one and `listener_py` on the other.
If you installed from the binary packages (i.e. you did not build it from source) then you'll need to invoke the examples with the Python executable directly (for now we're working on a fix for this).
Since you can also do this when building from source, we'll use this pattern in both cased, but you should be able to invoke the python scripts directly if you build from source.

To run them do `talker_py` and `listener_py` respectively, and you should see the following on the terminal running the listener:

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

Since `rclpy` uses the underlying ROS 2 infrastructure, Python programs that use `rclpy` can also communicate with applications written in C++, even if they use a different DDS vendor and are running on a different operating system.