## Overview
Client libraries are the APIs that allow users to implement their ROS code.
They are what users use to get access to ROS concepts such as nodes, topics, services, etc.
Client libraries come in a variety of programming languages so that users may write ROS code in the language that is best-suited for their application.
For example, you might prefer to write visualization tools in Python because it makes prototyping iterations faster, while for parts of your system that are concerned with efficiency, the nodes might be better implemented in C++.

Nodes written using different client libraries are able to share messages with each other because all client libraries implement code generators that provide users with the capability to interact with ROS interface files in the respective language.

In addition to the language-specific communication tools, client libraries expose to users the core functionality that makes ROS “ROS”.
For example, here is a list of functionality that can typically be accessed through a client library:
- Names and namespaces
- Time (real or simulated)
- Parameters
- Console logging
- Threading model
- Intra-process communication


## Common functionality: the RCL
Most of the functionality found in a client library is not specific to the programming language of the client library.
For example, the behavior of parameters and the logic of namespaces should ideally be the same across all programming languages.
Because of this, rather than implementing the common functionality from scratch, client libraries make use of a common core ROS Client Library (RCL) interface that implements logic and behavior of ROS concepts that is not language-specific.
As a result, client libraries only need to wrap common the functionality in the RCL with foreign function interfaces.
This keeps client libraries thinner and easier to develop.
For this reason the common RCL functionality is exposed with C interfaces as the C language is typically the easiest language for client libraries to wrap.

In addition to making the client libraries light-weight, an advantage of having the common core is that if any changes are made to the logic/behavior of the functionality in the core RCL -- namespaces, for example -- all client libraries that use the RCL will have these changes reflected.
This makes the client libraries easier to maintain when it comes to bug fixes.

[The API documentation for the RCL can be found here.](http://docs.ros2.org/beta1/api/rcl/)

The C++ client library (`rclcpp`) and the Python client library (`rclpy`) are both client libraries which utilize common functionality in the RCL.
For a walkthrough of the message exchange between a publisher using `rclpy` and a subscriber using `rclcpp`, we encourage you to watch [this video](https://vimeo.com/187696091) starting at 17:25 [(here are the slides)](http://roscon.ros.org/2016/presentations/ROSCon%202016%20-%20ROS%202%20Update.pdf).


## Language-specific functionality
Client library concepts that require language-specific features/properties are not implemented in the RCL but instead are implemented in each client library. 
For example, threading models used by “spin” functions will have implementations that are specific to the language of the client library.


## Summary
By utilizing the common core ROS client library, client libraries written in a variety of programming languages are easier to write and have more consistent behaviour.