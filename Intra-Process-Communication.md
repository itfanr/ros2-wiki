# Intra-process communication

## Background

ROS applications typically consist of a composition of individual "nodes" which perform narrow tasks and are decoupled from other parts of the system.
This promotes fault isolation, faster development, modularity, and code reuse, but it often comes at the cost of performance.
After ROS 1 was initially developed, the need for efficient composition of nodes became obvious and Nodelets were developed.
In ROS 2 we aim to improve on the design of Nodelets by addressing some fundamental problems that required restructuring of nodes.

In this demo we'll be highlighting how nodes can be composed manually, by defining the nodes separately but combining them in different process layouts without changing the node's code or limiting its abilities.

## Build the demos

These demos should work on any of the three major OSs (Windows, Mac, or Linux).
Some of them do require OpenCV to have been installed.

### Using the pre-built binaries

If you've installed the binaries, simply source the ROS 2 setup file and then skip down to any of the individual demos to see how to run them.

### Building from source

Make sure you have OpenCV installed and then follow the source instructions.
You can find the from source instructions linked from the main [ros2 installation page](https://github.com/ros2/ros2/wiki/Installation).
Once built source the setup file and continue down to one of the specific demos to read about them and for instructions on how to run them.

## Running and understanding the demos

There are a few different demos: some are toy problems designed to highlight features of the intra process communications functionality and some are end to end examples which use OpenCV and demonstrate the ability to recombine nodes into different configurations.

### The two node pipeline demo

This demo is designed to show that the intra process publish/subscribe connection can result in zero-copy transport of messages when publishing and subscribing with `std::unique_ptr`s.

First let's take a look at the source:

https://github.com/ros2/demos/blob/release-alpha5/intra_process_demo/src/two_node_pipeline/two_node_pipeline.cpp
```c++
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// Node that produces messages.
struct Producer : public rclcpp::Node
{
  Producer(const std::string & name, const std::string & output)
  : Node(name, true)
  {
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output, rmw_qos_profile_default);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub]() -> void {
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        static int32_t count = 0;
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = count++;
        printf(
          "Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
          reinterpret_cast<std::uintptr_t>(msg.get()));
        pub_ptr->publish(msg);
      };
    timer_ = this->create_wall_timer(1_s, callback);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Node that consumes messages.
struct Consumer : public rclcpp::Node
{
  Consumer(const std::string & name, const std::string & input)
  : Node(name, true)
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input, [](std_msgs::msg::Int32::UniquePtr msg) {
      printf(
        " Received message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
    }, rmw_qos_profile_default);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto producer = std::make_shared<Producer>("producer", "number");
  auto consumer = std::make_shared<Consumer>("consumer", "number");

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();
  return 0;
}
```

As you can see by looking at the `main` function, we have a producer and a consumer node, we add them to a single threaded executor, and then call spin.

If you look at the "producer" node's implementation in the `Producer` struct, you can see that we have created a publisher which publishes on the "number" topic and a timer which periodically creates a new message, prints out its address in memory and its content's value and then publishes it.

The "consumer" node is a bit simpler, you can see its implementation in the `Consumer` struct, as it only subscribes to the "number" topic and prints the address and value of the message it receives.

The expectation is that the producer will print out an address and value and the consumer will print out a matching address and value.
This demonstrates that intra process communication is indeed working and unnecessary copies are avoided, at least for simple graphs.

Let's run the demo by executing the `two_node_pipeline` executable (don't forget to source the setup file first):

```
$ two_node_pipeline
Published message with value: 0, and address: 0x7fb02303faf0
Published message with value: 1, and address: 0x7fb020cf0520
 Received message with value: 1, and address: 0x7fb020cf0520
Published message with value: 2, and address: 0x7fb020e12900
 Received message with value: 2, and address: 0x7fb020e12900
Published message with value: 3, and address: 0x7fb020cf0520
 Received message with value: 3, and address: 0x7fb020cf0520
Published message with value: 4, and address: 0x7fb020e12900
 Received message with value: 4, and address: 0x7fb020e12900
Published message with value: 5, and address: 0x7fb02303cea0
 Received message with value: 5, and address: 0x7fb02303cea0
[...]
```

One thing you'll notice is that the messages tick along at about one per second.
This is because we told the timer to fire at about once per second.

Also you may have noticed that the first message (with value `0`) does not have a corresponding "Received message ..." line.
This is because publish/subscribe is "best effort" and we do not have any "latching" like behavior enabled.
This means that if the publisher publishes a message before the subscription has been established, the subscription will not receive that message.
This race condition can result in the first few messages being lost.
In this case, since they only come once per second, usually only the first message is lost.

Finally, you can see that "Published message..." and "Received message ..." lines with the same value also have the same address.
This shows that the address of the message being received is the same as the one that was published and that it is not a copy.
This is because we're publishing and subscribing with `std::unique_ptr`s which allow ownership of a message to be moved around the system safely.
You can also publish and subscribe with `const &` and `std::shared_ptr`, but zero-copy will not occur in that case.

### The cyclic pipeline demo

This demo is similar to the previous one, but instead of the producer creating a new message for each iteration, this demo only ever uses one message instance.
This is achieved by creating a cycle in the graph and "kicking off" communication by externally making one of the nodes publish before spinning the executor:

https://github.com/ros2/demos/blob/release-alpha5/intra_process_demo/src/cyclic_pipeline/cyclic_pipeline.cpp
```c++
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// This node receives an Int32, waits 1 second, then increments and sends it.
struct IncrementerPipe : public rclcpp::Node
{
  IncrementerPipe(const std::string & name, const std::string & in, const std::string & out)
  : Node(name, true)
  {
    // Create a publisher on the output topic.
    pub = this->create_publisher<std_msgs::msg::Int32>(out, rmw_qos_profile_default);
    std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> captured_pub = pub;
    // Create a subscription on the input topic.
    sub = this->create_subscription<std_msgs::msg::Int32>(
      in, [captured_pub](std_msgs::msg::Int32::UniquePtr msg) {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr) {
        return;
      }
      printf(
        "Received message with value:         %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
      printf("  sleeping for 1 second...\n");
      if (!rclcpp::sleep_for(1_s)) {
        return;    // Return if the sleep failed (e.g. on ctrl-c).
      }
      printf("  done.\n");
      msg->data++;    // Increment the message's data.
      printf(
        "Incrementing and sending with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
      pub_ptr->publish(msg);    // Send the message along to the output topic.
    }, rmw_qos_profile_default);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create a simple loop by connecting the in and out topics of two IncrementerPipes.
  // The expectation is that the address of the message being passed between them never changes.
  auto pipe1 = std::make_shared<IncrementerPipe>("pipe1", "topic1", "topic2");
  auto pipe2 = std::make_shared<IncrementerPipe>("pipe2", "topic2", "topic1");
  rclcpp::sleep_for(1_s);  // Wait for subscriptions to be established to avoid race conditions.
  // Publish the first message (kicking off the cycle).
  std::unique_ptr<std_msgs::msg::Int32> msg(new std_msgs::msg::Int32());
  msg->data = 42;
  printf(
    "Published first message with value:  %d, and address: 0x%" PRIXPTR "\n", msg->data,
    reinterpret_cast<std::uintptr_t>(msg.get()));
  pipe1->pub->publish(msg);

  executor.add_node(pipe1);
  executor.add_node(pipe2);
  executor.spin();
  return 0;
}
```

Unlike the previous demo, this demo uses only one Node, instantiated twice with different names and configurations.
The graph ends up being `pipe1` -> `pipe2` -> `pipe1` ... in a loop.

The line `pipe1->pub->publish(msg);` kicks the process off, but from then on the messages are passed back and forth between the nodes by each one calling publish within its own subscription callback.

The expectation here is that the nodes pass the message back and forth, once a second, incrementing the value of the message each time.
Because the message is being published and subscribed to as a `unique_ptr` the same message created at the beginning is continuously used.

To test those expectations, let's run it:

```
% cyclic_pipeline
Published first message with value:  42, and address: 0x7fd2ce0a2bc0
Received message with value:         42, and address: 0x7fd2ce0a2bc0
  sleeping for 1 second...
  done.
Incrementing and sending with value: 43, and address: 0x7fd2ce0a2bc0
Received message with value:         43, and address: 0x7fd2ce0a2bc0
  sleeping for 1 second...
  done.
Incrementing and sending with value: 44, and address: 0x7fd2ce0a2bc0
Received message with value:         44, and address: 0x7fd2ce0a2bc0
  sleeping for 1 second...
  done.
Incrementing and sending with value: 45, and address: 0x7fd2ce0a2bc0
Received message with value:         45, and address: 0x7fd2ce0a2bc0
  sleeping for 1 second...
  done.
Incrementing and sending with value: 46, and address: 0x7fd2ce0a2bc0
Received message with value:         46, and address: 0x7fd2ce0a2bc0
  sleeping for 1 second...
  done.
Incrementing and sending with value: 47, and address: 0x7fd2ce0a2bc0
Received message with value:         47, and address: 0x7fd2ce0a2bc0
  sleeping for 1 second...
[...]
```

You should see ever increasing numbers on each iteration, starting with 42... because 42, and the whole time it reuses the same message, as demonstrated by the pointer addresses which do not change, which avoids unnecessary copies.

### The image pipeline demo

In this demo we'll use OpenCV to capture, annotate, and then view images.

Note for OS X users: If you these examples do not work or you receive an error like `ddsi_conn_write failed -1` then you'll need to increase your system wide UDP packet size:

```
$ sudo sysctl -w net.inet.udp.recvspace=209715
$ sudo sysctl -w net.inet.udp.maxdgram=65500
```

These changes will not persist after a reboot.

#### Simple pipeline

First we'll have a pipeline of three nodes, arranged as such: `camera_node` -> `watermark_node` -> `image_view_node`

The `camera_node` reads from camera device `0` on your computer, writes some information on the image and publishes it.
The `watermark_node` subscribes to the output of the `camera_node` and adds more text before publishing it too.
Finally, the `image_view_node` subscribes to the output of the `watermark_node`, writes more text to the image and then visualizes it with `cv::imshow`.

In each node the address of the message which is being sent, or which has been received, or both, is written to the image.
The watermark and image view nodes are designed to modify the image without copying it and so the addresses imprinted on the image should all be the same as long as the nodes are in the same process and the graph remains organized in a pipeline as sketched above.

**Note:** On some systems (we've seen it happen on Linux), the address printed to the screen might not change.
This is because the same unique pointer is being reused.
In this situation, the pipeline is still running.

Let's run the demo by executing the `image_pipeline_all_in_one` executable, and you should see something like this:

![](http://i.imgur.com/tqiIVgT.png)

You can pause the rendering of the image by pressing the spacebar and you can resume by pressing the spacebar again.
You can also press `q` or `ESC` to exit.

If you pause the image viewer, you should be able to compare the addresses written on the image and see that they are the same.

#### Pipeline with two image viewers

Now let's look at an example just the one above, except it has two image view nodes.
All the nodes are still in the same process, but now two image view windows should show up. (Note for OS X users: your image view windows might be on top of each other.)
Let's run it with the command `image_pipeline_with_two_image_view`:

![](http://i.imgur.com/iLIT02t.png)

Just like the last example, you can pause the rendering with the spacebar and continue by pressing the spacebar a second time. You can stop the updating to inspect the pointers written to the screen.

As you can see in the example image above, we have one image with all of the pointers the same and then another image with the same pointers as the first image for the first two entries, but the last pointer on the second image is different. To understand why this is happening consider the graph's topology:

`camera_node` -> `watermark_node` -> `image_view_node`
                                 \-> `image_view_node2`

The link between the `camera_node` and the `watermark_node` can use the same pointer without copying because there is only one intra process subscription to which the message should be delivered. But for the link between the `watermark_node` and the two image view nodes the relationship is one to many, so if the image view nodes were using `unique_ptr` callbacks then it would be impossible to deliver the ownership of the same pointer to both. It can be, however, delivered to one of them. Which one would get the original pointer is not defined, but instead is simply the last to be delivered.

Note that the image view nodes are not subscribed with `unique_ptr` callbacks. Instead they are subscribed with `const shared_ptr`s. This means the system could have delivered the same `shared_ptr` to both callbacks. Currently the intra process system is not that intelligent and so it stores the message internally as a `unique_ptr` and copies it into a `shared_ptr` for each callback until the last one. On the last callback, regardless of the type, the ownership is transferred out of intra process storage and, in the case of the image view, the ownership is moved into a new `shared_ptr` and delivered. Thus, one of the image view nodes gets a copy and the other gets the original.

#### Pipeline with interprocess viewer

One other important thing to get right is to avoid interruption of the intra process zero-copy behavior when interprocess subscriptions are made. To test this we can run the first image pipeline demo, `image_pipeline_all_in_one`, and then run an instance of the stand alone `image_view_node`. This will look something like this:

![](http://i.imgur.com/MoWRH1u.png)

It's hard to pause both images at the same time so the images may not line up, but the important thing to notice is that the `image_pipeline_all_in_one` image view shows the same address for each step. This means that the intra process zero-copy is preserved even when an external view is subscribed as well. You can also see that the interprocess image view has different process IDs for the first two lines of text and the process ID of the standalone image viewer in the third line of text.

## Looking forward

These demos are the foundation for some cool new features on which we're actively working, but right now some things are missing.

### Room for Improvement

Let's start by looking at what we at OSRF know we can do better or differently and move on from there.

#### Intra Process Manager Storage

At the core of the intra process implementation is something called the intra process manager. It is the shared state between nodes (not necessarily global) which facilitates intra process communication. The intra process has a lot of room for improvement, but one thing on our short list is to have it be more intelligent about how to store the user's data internally. In the example with two image view nodes all in the same process, we could have delivered the user's provided pointer to both image view nodes as copies of a single `shared_ptr`. This is possible only because of the intra process graph's structure, but given any relationship of a publisher to one or more intra process subscriptions there should be a preferred solution.

For example, imagine if there was a publisher connected to three intra process subscriptions where one was subscribed as a `unique_ptr` and the other two were subscribed as a `shared_ptr`. If you store the the message as a `unique_ptr` then you must make a copy for each of the `shared_ptr` but you can deliver to the `unique_ptr` callback without a copy. But if you instead stored the message as a `shared_ptr` then you give that `shared_ptr` to the two `shared_ptr` callbacks and make one copy for the `unique_ptr` callback, which would save on copies. In both cases we've assumed that only one copy of the message should be stored, i.e. we will not store a `unique_ptr` of the message as well as a `shared_ptr` copy. There is a performance trade-off when decided whether to make those copies or not, so one thing to figure out moving forward is how and when to expose this trade-off to the developer.

This problem gets more interesting when we start doing Type Masquerading :smile:, which we'll talk about below.

#### Avoiding Unnecessary Interprocess Publishes

Currently we're relying on the middleware, the DDS vendor, to avoid publishing to the wire unnecessarily, but no matter what we have to give the middleware a copy of the user's message. We could avoid this copy given to the system if we could know if it is needed. We can do this currently by checking to see if there are any non intra process subscriptions currently attached to the publisher. The problem with this becomes apparent when we go to implement latching, which we'll see more about below.

#### Avoiding Memory Allocation

In other parts of the system we've worked really hard to allow users to avoid memory allocation. This has performance benefits and may be required for real-time or embedded scenarios. We cannot currently do that with intra process. Mostly this is because we haven't had time to figure out the right interfaces, but the general problem is that if a message needs to be delivered to more than one subscription, or if the user gives us a `const &` or `const shared_ptr` when publishing, we need to make a copy. And the destination of the copy is currently created using `new` and is not configurable. We expect to resolve that in the future.

#### Performance, Performance, Performance

This is a very rough first draft. There is a lot of room for improvement, even beyond what has been enumerated above. We'll start to improve performance as we dig into the details of the system, build up a better understanding of exactly what our middleware vendors are doing, and try alternative strategies for implementing intra process.

### What's Missing

Aforementioned are some things we can improve with what's already there. But there are also some things we'd like to add on top of this that are pretty interesting and some things that are just necessary.

#### Latching

We haven't fully implemented the concept of latching yet, but it's very likely we'll need to adjust the implementation of the intra process manager to account for the fact that late intra process subscriptions should be delivered to as well. There are several options on how to do that, and we'll do some testing and figure out what to do in the near future.

#### Beyond Pub/Sub

We've not done any of this with Services, Parameters, or Actions, but we will.

#### Type Masquerading

This is one of the coolest upcoming features that we didn't get to in this demo.
Imagine the image pipeline demo above, but rather than passing `sensor_msgs/Image`s around, you're publishing and subscribing to `cv::Mat` objects. This exists in ROS 1, see: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes

In ROS 1, this is accomplished by serializing/deserializing the third party type when handling it. This means that with intra process you'll be serializing when passing it between nodelets. But in ROS 2 we want to do it in the most performant way possible. Similar to how these demos have been demonstrating that an instance of a message can be used through the whole pipeline in certain cases, we'd like to do the same with third party types. So conceivably you could have the image pipeline with a single `cv::Mat` which never gets copied by the middleware. To do this requires some additional intelligence in the intra process manager, but we've already got a design and some proof of concepts in the works.

Given these features, hopefully there will come a point where you can trust the middleware to handle your data as efficiently as is possible. This will allow you to write performant algorithms without sacrificing modularity or introspection!

#### Tooling

One of the sticking points of Nodelets in ROS 1 was the complexity of defining, building, and using them.
We've not tackled that problem yet, but we are working on it. We've got a port of class loader (https://github.com/ros/class_loader/tree/ros2) and pluginlib (https://github.com/ros/pluginlib/tree/ros2) for ROS 2, but we only have prototypes of the CMake infrastructure which will help users build and run their nodes. Here's a sketch of the design we expect:

```cmake
add_node(my_node src/my_node.cpp)
target_link_libraries(my_node ${external_dependency_LIBRARIES} ...)
```

We'll provide an interface similar to CMake's `add_executable` or `add_library`. This doesn't preclude the idea of providing a more automatic solution a la `ament_cmake_auto`.

This simple CMake entry will generate a few things:

- A marker file used to discover the node by pluginlib.
- A shared library for your node.
- An executable for your node.
 - The executable can run the node in its own process, or serve as a proxy while the node runs in a different container.

We'll also need to develop the container, which can run nodes inside of itself and is controlled externally by ROS primitives like Services.

We've got a lot of work to do, but hopefully this tutorial gives you a sense of where we're going and what we're trying to do in terms of performance and features.