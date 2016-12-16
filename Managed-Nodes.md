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
In order to run this demo, we open three terminals and source our ROS2 environment variables either from the binary distributions or the workspace we compiled from source.

|lifecycle_talker|lifecycle_listener|lifecycle_service_client|
|----------------|------------------|------------------------|
|```$ lifecycle_talker```| ```$ lifecycle_listener```|```$ lifecycle_service_client```|
|[![asciicast](https://asciinema.org/a/e0f11qvpberltp8r1w04wzw9t.png)](https://asciinema.org/a/e0f11qvpberltp8r1w04wzw9t)|[![asciicast](https://asciinema.org/a/442pjcu729t3vsld7n225orl7.png)](https://asciinema.org/a/442pjcu729t3vsld7n225orl7)|[![asciicast](https://asciinema.org/a/6o20wbnhx6tk3y2hr5dk8fwm5.png)](https://asciinema.org/a/6o20wbnhx6tk3y2hr5dk8fwm5)|

If we look at the output of the `lifecycle_talker`, we notice that nothing seems to happen. And this does make sense, since every node starts as `unconfigured`. The lifecycle_talker is not configured yet and in our example, no publishers and timers are created yet.
The same behavior can be seen for the `lifecycle_listener`, which is less surprising given that no publishers are available at this moment.
The interesting part starts with the third terminal. In there we launch our `lifecycle_service_client` which is responsible for changing the states of the `lifecycle_talker`. 

####Triggering transition 1 (configure)
```
[lc_client] Transition 1 successfully triggered.
[lc_client] Node lc_talker has current state inactive.
```
makes the lifecycle talker change its state to inactive. Inactive means that all publishers and timers are created and configured. However, the node is still not active. Therefore no messages are getting published.
```
[lc_talker] on_configure() is called.
Lifecycle publisher is currently inactive. Messages are not published.
...
```
The lifecycle listener on the same time receives a notification as it listens to every state change notification of the lifecycle talker. In fact, the listener receives two consecutive notifications. One for changing from the primary state "unconfigured" to "configuring". Because the configuring step was successful within the lifecycle talker, a second notification from "configuring" to "inactive". 
```
[lc_listener] notify callback: Transition from state unconfigured to configuring
[lc_listener] notify callback: Transition from state configuring to inactive
```

####Triggering transition 2 (activate)
```
[lc_client] Transition 2 successfully triggered.
[lc_client] Node lc_talker has current state active.
```
makes the lifecycle talker change its state to active. Active means that all publishers and timers are now activated. Therefore the messages are now getting published. 
```
[lc_talker] on_activate() is called.
[lc_talker] Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #11]
[lc_talker] Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #12]
...
```
The lifecycle listener receives the same set of notifications as before. Lifecycle talker changed its state from inactive to active.
```
[lc_listener] notify callback: Transition from state unconfigured to configuring
[lc_listener] notify callback: Transition from state configuring to inactive
```
The difference to the transition event before is that our listener now also receives the actual published data.
```
[lc_listener] data_callback: Lifecycle HelloWorld #11
[lc_listener] data_callback: Lifecycle HelloWorld #12
...
```
Please note that the index of the published message is already at 11. The purpose of this demo is that even though we call `publish` at every state of the lifecycle talker, only when the state in active, the messages are actually published. As for the beta1, all other messages are getting ignored. This behavior may change in future versions in order to provide better error handling.

For the rest of the demo, you will see similar output as we deactivate and activate the lifecycle talker and finally shut it down. 

##The demo code

####lifecycle_talker, lifecycle_listener and lifecycle_service_client
If we have a look at the code, there is one significant change for the lifecycle talker compared to a regular talker. Our node does not inherit from the regular ```rclcpp::node::Node``` but from ```rclcpp_lifecycle::LifecycleNode```.
```
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
```
Every child of LifecycleNodes have a set of callbacks provided. These callbacks go along with the applied state machine attached to it. These callbacks are:
* ```rcl_lifecycle_ret_t on_configure(const rclcpp_lifecycle::State & previous_state)```
* ```rcl_lifecycle_ret_t on_activate(const rclcpp_lifecycle::State & previous_state)```
* ```rcl_lifecycle_ret_t on_deactivate(const rclcpp_lifecycle::State & previous_state)```
* ```rcl_lifecycle_ret_t on_cleanup(const rclcpp_lifecycle::State & previous_state)```
* ```rcl_lifecycle_ret_t on_shutdown(const rclcpp_lifecycle::State & previous_state)```

All these callbacks have a positive default return value (```return RCL_LIFECYCLE_RET_OK```). This allows that a lifecycle node can change its state even though no explicit callback function was overwritten. 
There is one other callback function for error handling. Whenever a state transition throws an uncaught exception, we call ```on_error```. 
* ```rcl_lifecycle_ret_t on_error(const rclcpp_lifecycle::State & previous_state)```

This gives room for executing a custom error handling. Only (!) in the case that this function returns ```RCL_LIFECYCLE_RET_OK```, the state machine transitions to the state `unconfigured`. By default, the `on_error` returns `RCL_LIFECYCLE_RET_ERROR` and the state machine transitions into `finalized`. 

###lifecycle_service_client_py.py
The `lifecycle_service_client` application is a fixed order script for this demo purpose only. I explains the use and the API calls made for this lifecycle implementation, but may be inconvenient to use otherwise. For this reason, we implemented a separate python script, which lets you dynamically change states or various nodes.
```
python3 `which lifecycle_service_client_py.py`
usage: lifecycle_service_client_py.py [-h]
                                      [--change-state-args {configure,cleanup,shutdown,activate,deactivate}]
                                      {change_state,get_state,get_available_states,get_available_transitions}
                                      node
```
In the case you want to get the current state of the `lc_talker` node, you'd call:
```
$ python3 `which lifecycle_service_client_py.py` get_state lc_talker
lc_talker is in state unconfigured(1)
```
The next step would be to execute a state change:
```
$ python3 `which lifecycle_service_client_py.py` change_state --change-state-args configure lc_talker
```