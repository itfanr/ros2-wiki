# Overview

ROS 2 includes a suite of command-line tools for introspecting a ROS 2 system.

## Usage
The main entry point for the tools is the command `ros2`, which itself has various sub-commands for introspecting and working with nodes, topics, services, and more.

To see all available sub-commands run:

```
ros2 --help
```

_Note that passing arguments to a node run with `ros2 run` currently requires passing `--` before the node arguments, e.g.:_

```
ros2 run demo_nodes_cpp talker -- -t chatter2
```

## Example

To produce the typical talker-listener example using command-line tools, the `topic` sub-command can be used to publish and echo messages on a topic.

Publish messages in one terminal with:
```
$ ros2 topic pub /chatter std_msgs/String "data: Hello world"
publisher: beginning loop
publishing std_msgs.msg.String(data='Hello world')

publishing std_msgs.msg.String(data='Hello world')
```

Echo messages received in another terminal with:
```
$ ros2 topic echo /chatter
data: Hello world

data: Hello world
```

## Behind the scenes

ROS 2 uses a distributed discovery process for nodes to connect to each other.
As this process purposefully does not use a centralized discovery mechanism (like the ROS Master in ROS 1), it can take time for ROS nodes to discover all other participants in the ROS graph.
Because of this, there is a long-running daemon in the background that stores information about the ROS graph to provide faster responses to queries, e.g. the list of node names.

The daemon is automatically started when the relevant command-line tools are used for the first time.
You can run `ros2 daemon --help` for more options for interacting with the daemon.

## Implementation

The source code for the `ros2` command is available at https://github.com/ros2/ros2cli

The `ros2` tool has been implemented as a framework that can be extended via plugins.
For example, [the `sros2` package](https://github.com/ros2/sros2) provides a `security` sub-command that is automatically detected by the `ros2` tool if the `sros2` package is installed.