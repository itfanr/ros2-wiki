# Composable nodes as shared libraries

**Context**

You want to export composable nodes as a shared libraries from some packages and using those in another package that does link-time composition.

**Solution**

* add code to the CMake file which imports the actual targets in downstream packages
  * install the generated file
  * export the generated file

**Example**

[ROS Discourse - Ament best practice for sharing libraries](https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602)