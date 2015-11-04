# ROS 2 alpha2 release (code name *Baling wire*; October 2015)

Welcome to the second release of ROS 2 software!  We hope that you try it out and [provide feedback](#contact-us).

## Table of Contents

- [Background](#background)
- [Status](#status)
- [Intended audience](#intended-audience)
- [Scope](#scope)
- [Contact us](#contact-us)

## Background

As explained in a [design
article](http://design.ros2.org/articles/why_ros2.html), we are engaged in
the development of a new major version of ROS, called "ROS 2." While the
underlying concepts (e.g., publish / subscribe messaging) and goals (e.g.,
flexibility and reusability) are the same as for ROS 1, we are taking this
opportunity to make substantial changes to the system, including changing
some of the core APIs.
For a deeper treatment of those changes and their
rationale, consult the other [ROS 2 design
articles](http://design.ros2.org).

## Status

On October 30, 2015, we are releasing ROS 2 alpha2,
code-named **Baling wire**.
Our primary goal with this release is to add more features, while also addressing the feedback we received for the previous alpha 1 release.
To that end, we built a set of [demos](Tutorials) that
show some of the key features of ROS 2.
We encourage you to try out those
demos, look at the code that implements them, and [provide
feedback](#contact-us).
We're especially interested to know how well (or
poorly) we're addressing use cases that are important to you.

## Intended audience

While everyone is welcome to try out the demos and look through the code, we're aiming this release at people who are already experienced with ROS 1 development.
At this point, the ROS 2 documentation is pretty sparse and much of the system is explained by way of how it compares to ROS 1.

## Scope

As the "alpha" qualifier suggests, this release of ROS 2 is far from
complete.
You should not expect to switch from ROS 1 to ROS 2, nor should
you expect to build a new robot control system with ROS 2.
Rather, you
should expect to try out some demos, explore the code, and perhaps write
your own demos.

The major features included in this release are:

- Refactor rclcpp into a library
- Support for custom allocators in rclcpp, useful for real-time messaging
- Feature parity of Windows with Linux/OSX, including workspace management, services and parameters
- rclcpp API improvements
- FreeRTPS improvements

Pretty much anything not listed above is not included in this release.
The next steps are described in the [[Roadmap]].

## Contact us

See [[the contact page|Contact]].