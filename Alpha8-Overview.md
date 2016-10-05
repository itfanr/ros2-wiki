# ROS 2 alpha8 release <br\> (code name *Hook-and-Loop*; October 2016)

Welcome to the latest release of ROS 2 software!  We hope that you try it out and [provide feedback](#contact-us).

## Changes to supported DDS vendors

Until Alpha 8, ROS 2 was supporting ROS middleware implementations for eProsima's Fast RTPS, RTI's Connext and PrismTech's OpenSplice.
To streamline our efforts, as of Alpha 8, Fast RTPS and Connext (static) will be supported, with Fast RTPS ([now Apache 2.0-licensed](http://www.eprosima.com/index.php/company-all/news/61-eprosima-goes-apache)) shipped as the default.

## Scope

As the "alpha" qualifier suggests, this release of ROS 2 is far from complete.
You should not expect to switch from ROS 1 to ROS 2, nor should you expect to build a new robot control system with ROS 2.
Rather, you should expect to try out some demos, explore the code, and perhaps write your own demos.

The improvements included in this release are:

- Several improvements to Fast RTPS and its rmw implementation
  - Support for large (image) messages in Fast RTPS
  - `wait_for_service` functionality in Fast RTPS
- Support for all ROS 2 message types in Python and C
- Added support for Quality of Service (QoS) settings in Python
- Fixed various bugs with the previous alpha release

Pretty much anything not listed above is not included in this release.
The next steps are described in the [[Roadmap]].

## Contact us

See [[the contact page|Contact]].