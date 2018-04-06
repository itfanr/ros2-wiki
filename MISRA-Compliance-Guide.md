This section tries to give guidance about how to integrate ROS2 into a system that is intended to be copmliant with the MISRA (Motor Industry Software Reliability Association) guidelines.

**What this section is about:**
- ROS2 core packages
- ROS2 core client libraries
- Integration considerations for ROS2 packages in a MISRA-Compliant system

**What this section is not about:**
- Applying MISRA Guidelines to application and ecosystem ROS2 packages.
- A detailed description of the MISRA Guidelines

**Relation to other sections of this wiki:**

- The [Quality Guide](https://github.com/ros2/ros2/wiki/Quality-Guide) summarizes overall techniques and strategies for producing high quality ROS2 packages.  

## What are the MISRA guidelines?

From [MISRA](https://www.misra.org.uk/Activities/MISRAC/tabid/160/Default.aspx)
> MISRA was originally established as a collaboration between vehicle manufacturers, component suppliers and engineering consultancies, and seeks to promote best practice in developing safety-related electronic systems in road vehicles and other embedded systems. To this end MISRA publishes documents that provide accessible information for engineers and management, and holds events to permit the exchange of experiences between practitioners.

MISRA publishes a set of guidelines for both C and C++ that define a subset of the languages that are likely to be free from important programming mistakes for safety-critical systems. The MISRA guidelines are used as a component of [various software standards](https://en.wikipedia.org/wiki/MISRA_C#Adoption), such as:

* [ISO 26262](https://en.wikipedia.org/wiki/ISO_26262) - "Road Vehicles - Functional Safety"
* [AUTOSAR](https://en.wikipedia.org/wiki/AUTOSAR) - Automotive Open System Architecture

## Why is this important to ROS2 users?

As robotics and autonomy grow, especially in the field of self-driving cars, users of ROS will need to be able to determine if the software is able to be used in a safety-critical environment. With suitable guidance and modification, it is expected that ROS2 could be integrated as part of a MISRA compliant system. This would enable users of ROS2 to take their work through multiple stages of the software lifecyce, from prototype through production.
