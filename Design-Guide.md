# Composable nodes as shared libraries

**Context**

You want to export composable nodes as a shared libraries from some packages and using those in another package that does link-time composition.

**Solution**

* add code to the CMake file which imports the actual targets in downstream packages
  * install the generated file
  * export the generated file

**Example**

[ROS Discourse - Ament best practice for sharing libraries](https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602)

# FastRTPS large data transfer

**Context**

You want to transfer large data via FastRTPS.

**Problem**

DDS/RTPS uses UDP with a maximum message size of 64k

**Solution**

configure the middleware that it fragements large data into messages

**Implementation**

use Asynchronous publication mode:

```
<publishMode>
  <kind>ASYNCHRONOUS</kind>
</publishMode>
```

[ROS2 Fine Tuning](https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf)