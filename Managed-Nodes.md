##Introduction
ROS2 introduces the concept of managed nodes, also called `LifecycleNode`s. In the following tutorial, we explain the purpose of these nodes, what makes them different from regular nodes and how they comply to a lifecycle management.
Managed nodes are scoped within a state machine of a finite amount of states. These states can be changed by invoking a transition id which indicates the succeeding consecutive state.
The state machine is implemented as described at the [ROS2 design page](http://design.ros2.org/articles/node_lifecycle.html) 
Our implementation differentiates between `Primary States` and `Transition States`. Primary States are supposed to be steady states in which any node can do the respected task. On the other hand, Transition States are meant as temporary intermediate states in between transitions. The result of these intermediate states are used to indicate whether a transition between two primary states is considered successful or not. Thus, any managed node can be in one of the following states:

Primary States (steady states):
* unconfigured
* inactive
* active
* shutdown

Transition States (intermediate states):
* configuring
* activating
* deactivating
* cleaningup
* shuttingdown

The possible transitions to invoke are:
* configure
* activate
* deactivate
* cleanup
* shutdown

For a more verbose explanation on the applied state machine, we refer to the design page which provides an in-detail explanation about each state and transition.

##The demo
###What's happening
The demo is split into 3 different separate applications.
* lifecycle_talker
* lifecycle_listener
* lifecycle_service_client 

The `lifecycle_talker` represents a managed node and publishes according to which state the node is in. We split the tasks of the talker node into separate pieces and assign them in the respective state.
 1. configuring: We initialize our publisher and timer
 2. activate: We activate the publisher and timer in order to enable a publishing
 3. deactivate: We stop the publisher and timer
 4. cleanup: We destroy the publisher and timer

The principle is implemented in this demo as the typical talker/listener demo. However, imaging a real scenario with attached hardware, one could image bringing up the device driver in the configuring state, start and stop only the publishing of the device's data and only in the cleanup/shutdown phase actually shutdown the device. 

The `lifecycle_listener` is a simple listener which shows the characteristics of the lifecycle talker. The talker enables the message publishing only in the active state and thus making the listener receiving only messages when the talker is in an active state.

The `lifecycle_service_client` is a script calling different transitions on the `lifecycle_talker`. This is meant as the external user controlling the lifecycle of nodes.   

##Run the demo

##The demo code