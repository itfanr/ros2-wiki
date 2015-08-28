#Background
ROS 2 is built on the DDS middleware, which offers a rich variety of quality of service parameters. With the right set of quality of service parameters, DDS can be as reliable as TCP or as best-effort as TCP, with many, many possible states in between. Unlike ROS 1, which primarily only supported TCP, ROS 2 will benefit from the flexibility of DDS in environments with lossy wireless networks where a best effort policy would be more suitable, or in real-time computing systems where the right quality of service profile is needed to meet deadlines.

The flexibility offered by DDS comes at the cost of complexity. The philosophy of ROS 2 will be to expose the quality of service parameters that ROS users and roboticists will care about, but not to duplicate the large number of quality of service options provided by DDS. (To learn more about DDS QoS policies, visit http://www.opendds.org/qospolicies.html or https://community.rti.com/documentation.)

In the quality of service demo, we will spawn a node that publishes a camera image and another that subscribes to the image and shows it on the screen. We will then simulate a lossy network connection between them and show how different quality of service settings handle the bad link.

#Build/install the demo
##From pre-compiled binaries
Simply download the binary packages for your OS from the [installation page](https://github.com/ros2/ros2/wiki/Installation).

##From source
OpenCV is a prerequisite for the QoS demo. See the [OpenCV documentation](http://docs.opencv.org/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html#table-of-content-introduction) for installation instructions.
Follow the instructions on the [installation page](https://github.com/ros2/ros2/wiki/Installation#building-from-source) for your particular platform.

#Run the demo
Before running the demo, make sure you have a working webcam connected to your computer.

Once you've installed ROS 2, source your setup.bash file:

```
. <path to ROS 2>/install/setup.bash
```

Then run:
```
showimage
```
Nothing will happen yet. `showimage` is a subscriber node that is waiting for a publisher on the `image` topic.

In a separate terminal, source the install file and run the publisher node:
```
cam2image
```
In this window, you'll see terminal output:
```
Publishing image #1
Publishing image #2
Publishing image #3
...
```

A window will pop up with the title "view" showing your camera feed. In the first window, you'll see output from the subscriber:
```
Received image #1
Received image #2
Received image #3
...
```

## The code explained
TODO

##Command line options
In one of your terminals, add a -h flag to the original command:
```
showimage -h
```
You'll see a list of the possible options you can pass to the demo.

`-h`: The help message.

`-r`: Reliability. There are two options for this policy: reliable or best effort. Reliable means that values may be reset and the underlying DDS publisher might block, in order for messages to get delivered in order. Best effort means that messages will get sent as is, and they may get dropped or lost without effecting the behavior of the publisher.

`-k`: History policy (the "k" stands for "keep"). Determines how DDS buffers messages in the time between the user code that called `publish` and the time when the message actually gets sent. There are two options for history: KEEP_ALL and KEEP_LAST. KEEP_ALL will buffer all messages before they get sent. KEEP_LAST limits the number of buffered messages to a depth specified by the user.

`-d`: Queue depth. Only used if the history policy is set to KEEP_LAST. The queue depth determines the maximum number of not yet received messages that get buffered on the sender's side before messages start getting dropped.

If you run `cam2image -h`, you'll see the same set of command line options and two extras:

`-s`: Toggle displaying the input camera stream. If you run `cam2image -s` by itself, you'll see a camera window. If you also run `showimage`, you'll see two camera windows.

`-x` and `-y`: Set the size of the camera feed (x sets the width, y sets the height).

The default quality of service settings are tuned for maximum reliability: the reliability policy is reliable, and the history policy is "keep all".

We won't see much of a difference if we change the quality of service settings, since the publisher and subscriber are passing messages over inter-process communication, and messages are unlikely to get dropped if they are travelling within the same machine.

##Add network traffic
TODO