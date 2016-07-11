# ROS 2 alpha7 release <br\> (code name *Glue Gun*; July 2016)

Welcome to the latest release of ROS 2 software!  We hope that you try it out and [provide feedback](#contact-us).

## New version of Ubuntu required

Until Alpha 6 ROS 2 was targeting Ubuntu Trusty Tahr (14.04). As of this Alpha ROS 2 is targeting Ubuntu Xenial Xerus (16.04) to benefit from newer versions of the compiler, CMake, Python, etc.

## Scope

As the "alpha" qualifier suggests, this release of ROS 2 is far from complete.
You should not expect to switch from ROS 1 to ROS 2, nor should you expect to build a new robot control system with ROS 2.
Rather, you should expect to try out some demos, explore the code, and perhaps write your own demos.

The major features included in this release are:

- Graph API functionality: wait_for_service
  - Added interfaces in rclcpp and make use of them in examples, demos, and tests
- Improved support for large messages in both Connext and Fast-RTPS (partial for Fast-RTPS)
- Turtlebot demo using ported code from ROS 1
  - See: https://github.com/ros2/turtlebot2_demo

Pretty much anything not listed above is not included in this release.
The next steps are described in the [[Roadmap]].

## Contact us

See [[the contact page|Contact]].