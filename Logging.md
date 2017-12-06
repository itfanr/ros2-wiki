# Logging and logger configuration

The logging functionality currently supported is:
- Client libraries (`rclcpp` and `rclpy`) using a common logging library to provide:
  - Log calls with a variety of filters.
  - Hierarchy of loggers.
  - Loggers associated with nodes that automatically use the node's name.
    - Automatic use of the node's namespace is forthcoming.
- Console output.
  - File output and functionality akin to [`rosout`](http://wiki.ros.org/rosout) for remote consumption of messages is forthcoming.
- Programmatic configuration of logger levels.
  - Config files and runtime configuration is forthcoming.

## Logger concepts

Log messages have a severity level associated with them: `DEBUG`, `INFO`, `WARN`, `ERROR` or `FATAL`, in ascending order.

A logger will only process log messages with severity at or higher than a specified level chosen for the logger.

Each node (in `rclcpp` and `rclpy`) has a logger associated with it that automatically includes the node's name.
If the node's name is externally remapped to something other than what is defined in the source code, it will be reflected in the logger name.
Automatic use of the node's namespace is forthcoming.
Non-node loggers can also be created that use a specific name.

Logger names represent a hierarchy.
If the level of a logger named "abc.def" is unset, it will defer to the level of its parent named "abc", and if that level is also unset, the default logger level will be used.
When the level of logger "abc" is changed, all of its descendants (e.g. "abc.def", "abc.ghi.jkl") will have their level impacted unless their level has been explicitly set.

## Logging usage

<!-- the `rclcpp` documentation won't include logging until we make a new doc dump -->
In C++:
- See the [logging demo](Logging-and-logger-configuration) for example usage.
- See the [`rclcpp` documentation]() for an extensive list of functionality.

In Python:
- See the [`rclpy` tests](https://github.com/ros2/rclpy/blob/master/rclpy/test/test_logging.py) for example usage.

## Logger configuration

Logger configuration is still under development.
For now, the severity level of individual loggers can be configured programmatically with, e.g.:

In C++:
```
rcutils_logging_set_logger_level("logger_name", RCUTILS_LOG_SEVERITY_DEBUG);
```

In Python:
```
logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
rclpy.logging.set_logger_level('logger_name', rclpy.logging.LoggingSeverity.DEBUG)
```

The [logging demo](Logging-and-logger-configuration) provides an example of manually exposing a service so that loggers can be configured externally; in the future we expect runtime configuration capabilities of loggers to be exposed automatically.

## Console output configuration

<!-- the rcutils docs won't mention this env var until we make a new doc dump -->
By default, console output will be formatted to include the message severity, logger name, and the message.
Information such as the file name, function name and line number of the log call are also available.
Custom console output format can be configured with the `RCUTILS_CONSOLE_OUTPUT_FORMAT` environment variable: see the [`rcutils` documentation for details](https://github.com/ros2/rcutils/blob/b804084ff0eac312f11c51849f4a5fcdaa4f153f/include/rcutils/logging.h#L49..L62).
As `rclpy` and `rclcpp` both use `rcutils` for logging, this will effect all Python and C++ nodes.