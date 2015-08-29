# Background

ROS applications typically consist of a composition of individual "nodes" which perform narrow tasks and are decoupled from other parts of the system.
This promotes fault isolation, faster development, modularity, and code reuse, but it often comes at the cost of performance.
After ROS 1 was initially developed, the need for efficient composition of nodes became obvious and Nodelets were developed.
In ROS 2 we aim to improve on the design of Nodelets by addressing some fundamental problems that required restructuring of nodes.

In this demo we'll be highlighting how nodes can be composed manually, by defining the nodes separately but combining them in different process layouts without changing the node's code or limiting its abilities.


# Build the demos

These demos should work on any of the three major OS's (Windows, Mac, or Linux).
Some of them do require OpenCV to have been installed.


## Using the pre-built binaries

If you've installed the binaries, simply source the ROS 2 setup file and then skip down to any of the individual demos to see how to run them.


## Building from source

Make sure you have OpenCV installed and then follow the from source instructions.
You can find the from source instructions linked from the main [ros2 repository](https://github.com/ros2/ros2.git).
Once built source the setup file and continue down to one of the specific demos to read about them and for instructions on how to run them.


# Running and understanding the demos

There are few different demos, some are toy problems designed to highlight features of the intra process communications functionality and some are end to end examples which use OpenCV and demonstrate the ability to recombine nodes into different configurations.


## The two node pipeline demo

This demo is designed to show that the intra process publish/subscribe connection can result in zero-copy transport of messages when publishing and subscribing with `std::unique_ptr`'s.

TODO(wjwwood): figure out how to perma link to the source
First lets take a look at the source:

https://github.com/ros2/demos/blob/intra_process_img/intra_process_comms/src/two_node_pipeline/two_node_pipeline.cpp

```c++
#include <chrono>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

struct Producer : public rclcpp::Node
{
  Producer(const std::string & output, const std::string & name = "producer")
  : Node(name, true)
  {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output, rmw_qos_profile_default);
    timer_ = this->create_wall_timer(1_s, [this]() {
      static size_t count = 0;
      std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
      msg->data = count++;
      printf("Published message with value: %d, and address: %p\n", msg->data, msg.get());
      pub_->publish(msg);
    });
  }

  rclcpp::Publisher::SharedPtr pub_;
  rclcpp::WallTimer::SharedPtr timer_;
};

struct Consumer : public rclcpp::Node
{
  Consumer(const std::string & input, const std::string & name = "consumer")
  : Node(name, true)
  {
    sub_ = this->create_subscription_with_unique_ptr_callback<std_msgs::msg::Int32>(
      input, rmw_qos_profile_default, [](std_msgs::msg::Int32::UniquePtr & msg) {
      printf(" Received message with value: %d, and address: %p\n", msg->data, msg.get());
    });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto producer = std::make_shared<Producer>("number");
  auto consumer = std::make_shared<Consumer>("number");

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();
  return 0;
}
```

As you can see by looking at the `main` function, we have a producer and a consumer node, we've added them to a single threaded executor and then spin.

If you look at the "producer" node's implementation in the `Producer` class, you can see that we have created a publisher which publishes on the "number" topic and a timer which periodically creates a new message, prints out it's address in memory and its content's value and then publishes it.

The "consumer" node is a bit simpler, you can see it's implementation in the `Consumer` class, as it only subscribes to the "number" topic and prints the address and value of the message it receives.

The expectation is that the producer will print out an address and value and the consumer will print out a matching address and value.
Demonstrating that intra process is indeed working and unnecessary copies are avoided, at least for simple graphs.

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

One thing you'll notice is that the message tick along at about one per second.
This is because we told the timer to fire at about once per second.

Also you may have noticed that the first message (with value `0`) does not have a corresponding "Received message ..." line.
This is because publish/subscribe is best effort and we do not have any "latching" like behavior enabled.
This means that if the publisher publishes a message before the subscription has been established, the subscription will not receive that message.
This race condition can result in the first few messages being lost.
In this case, since they only come once per second, usually only the first message is lost.

Finally, you can see that "Published message..." and "Received message ..." lines with the same value also have the same address.
This shows that the address of the message being received is the same as the one that was published and that it is not a copy.
This is because we're publishing and subscribing with `std::unique_ptr`'s which allow ownership of a message to be moved around the system safely.
You can also publish and subscribe with `const &` and `std::shared_ptr`, but zero-copy will not occur in that case.


## The cyclic pipeline demo

This demo is similar to the previous one, but instead of the producer creating a new message for each iteration, this demo only ever uses one message instance.
This is achieved by creating a cycle in the graph and "kicking off" communication by externally making one of the nodes publish before spinning the executor:

https://github.com/ros2/demos/blob/intra_process_img/intra_process_comms/src/cyclic_pipeline/cyclic_pipeline.cpp

```c++
#include <chrono>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// This node receives an Int32, waits 1 second, then increments and sends it.
struct IncrementerPipe : public rclcpp::Node
{
  IncrementerPipe(const std::string & name, const std::string & in, const std::string & out)
  : Node(name, true)
  {
    pub = this->create_publisher<std_msgs::msg::Int32>(out, rmw_qos_profile_default);
    sub = this->create_subscription_with_unique_ptr_callback<std_msgs::msg::Int32>(
      in, rmw_qos_profile_default,
      [this](std_msgs::msg::Int32::UniquePtr & msg) {
        printf("Received message with value:         %d, and address: %p\n", msg->data, msg.get());
        printf("  sleeping for 1 second...\n");
        if (!rclcpp::sleep_for(1_s)) {
          return;                               // Return if the sleep failed (e.g. ctrl-c).
        }
        printf("  done.\n");
        msg->data++;
        printf("Incrementing and sending with value: %d, and address: %p\n", msg->data, msg.get());
        this->pub->publish(msg);
      });
  }

  rclcpp::Publisher::SharedPtr pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create a simple loop by connecting the in and out topics of two IncrementerPipe's.
  // The expectation is that the address of the message being passed between them never changes.
  auto pipe1 = std::make_shared<IncrementerPipe>("pipe1", "topic1", "topic2");
  auto pipe2 = std::make_shared<IncrementerPipe>("pipe2", "topic2", "topic1");
  rclcpp::sleep_for(1_s);  // Wait for subscriptions to be established.
  // Publish the first message (kicking off the cycle).
  std::unique_ptr<std_msgs::msg::Int32> msg(new std_msgs::msg::Int32());
  msg->data = 42;
  printf("Published first message with value:  %d, and address: %p\n", msg->data, msg.get());
  pipe1->pub->publish(msg);

  executor.add_node(pipe1);
  executor.add_node(pipe2);
  executor.spin();
  return 0;
}
```

Unlike the previous demo, this demo uses only one Node instantiated twice with different names and configurations.
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

As you can see you'll ever increasing numbers on each iteration, start with 42... because 42, and the whole time it reuses the same message, as demonstrated by the pointer addresses which do not change, which avoids unnecessary copies.


## Other Toy Demos

TODO Other demo ideas:
- intra process with a fork to show copying
- demonstrate use of `shared_ptr` and the effect on zero-copy


## The image pipeline demo

This demo is meant to be a bit more concrete and relatable.
In this demo we have a pipeline of three nodes, arranged as such: `camera_node` -> `watermark_node` -> `image_view_node`

The `camera_node` reads from camera device `0` on your computer, writes some information on the image and publishes it.
The `watermark_node` subscribes to the output of the `camera_node` and adds more text before publishing it too.
Finally, the `image_view_node` subscribes to the output of the `watermark_node`, writes more text to the image and then visualizes it with `cv::imshow`.

In each node the address of the message which is being sent, or which has been received, or both is written to the image.
The watermark and image view nodes are designed to modify the image without copying it and so the addresses imprinted on the image should all be the same as long as the nodes are in the same process and the graph remains organized in a pipeline as sketched above.

**Note:** On some systems (we've seen it happen on Linux), the address printed to the screen might not change.
This is because the same unique pointer is being reused.
In this situation, the pipeline is still running.

Let's run the demo by executing the `image_pipeline_all_in_one` executable, and you should see something like this:

![](http://i.imgur.com/CtkGLsN.png)

You can pause the rendering of the image by pressing the spacebar and you can resume by pressing the spacebar again.
You can also press `q` or `ESC` to exit.

If you pause the image viewer, you should be able to compare the addresses written on the image and see that they are the same.

TODO(wjwwood): do some other process layout for comparison.
