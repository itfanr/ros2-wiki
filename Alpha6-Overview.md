# ROS 2 alpha6 release <br\> (code name *Fastener*; June 2016)

Welcome to the latest release of ROS 2 software!  We hope that you try it out and [provide feedback](#contact-us).

## Scope

As the "alpha" qualifier suggests, this release of ROS 2 is far from
complete.
You should not expect to switch from ROS 1 to ROS 2, nor should
you expect to build a new robot control system with ROS 2.
Rather, you
should expect to try out some demos, explore the code, and perhaps write
your own demos.

The major features included in this release are:

- Graph API functionality: wait_for_service
  - Added graph guard condition to nodes for waiting on graph changes
  - Added `rmw_service_server_is_available` for verifying if a service is available
- Refactored `rclcpp` to use `rcl`
- Improved support for complex message types in Python
  - Nested messages
  - Arrays
  - Strings

Pretty much anything not listed above is not included in this release.
The next steps are described in the [[Roadmap]].

## Contact us

See [[the contact page|Contact]].