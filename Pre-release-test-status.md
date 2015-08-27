Things to be tested are listed below.
Add your name to claim an item and strike it through / check it off when it's done.
Feel free to add more items here.

* Review pending pull requests (not testing, but we need to get everything merged)
  * List PRs here that need review
    * https://github.com/ros2/rclcpp/pull/97
      * in review
    * https://github.com/ros2/demos/pull/14
      * in review
    * https://github.com/ros2/rmw_opensplice/pull/74
      * pending comments
    * https://github.com/ros2/rmw_connext/pull/89
      * ?
    * https://github.com/ros2/rclcpp/pull/98
      * +1, ready to merge
    * ~~https://github.com/ros2/rmw/pull/36~~
      * not critical for release
    * https://github.com/ros2/system_tests/pull/38
      * +1, ready to merge
* Create up-to-date packages from a known state (e.g. create .repos file with hashed as gist)
* From-binary installation (best to start with a clean VM):
  * Windows
  * Linux
  * OSX
* From-source installation (best to start with a clean VM):
  * Windows
  * Linux **Tully**
  * OSX
* Demos:
  * real-time
    * Linux **Tully**
  * intra-process
    * Linux
    * OSX
    * Windows
  * embedded (at least one person should grab hardware from Morgan and try the whole thing)
    * Linux
    * no-OS
  * ROS 1 bridge
    * OSX
    * Linux
  * QoS
    * Linux
    * OSX
    * Windows
* Stuff not covered by the demos:
  * services - **Esteve**
  * parameters - hopefully fixed by services
  * documentation entry point - **Brian**
