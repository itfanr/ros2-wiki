Things to be tested are listed below.
Add your name to claim an item and strike it through / check it off when it's done.
Feel free to add more items here.

* Review pending pull requests (not testing, but we need to get everything merged)
  * List PRs here that need review

* Create up-to-date packages from a known state (e.g. create .repos file with hashed as gist)
  * ~~RC 1 was built without `CMAKE_BUILD_TYPE` being set~~
  * RC 2 was built with `CMAKE_BUILD_TYPE=RelWithDebInfo`
* From-binary installation (best to start with a clean VM):
  * Windows
      * **Esteve**
  * Linux
    * **Tully**
    * **Steve** installed with docker `osrf/ros:indigo-desktop` image. :heavy_check_mark:
  * OSX
      * **William** :shipit: 
* From-source installation (best to start with a clean VM) using https://gist.github.com/dirk-thomas/25906bd5af20e64d311c:
  * Windows
  * Linux
    * OpenSplice OSRF Debs - **Morgan** :heavy_check_mark:
      * **Tully** :heavy_check_mark:
    * OpenSplice binaries - **Morgan** :heavy_check_mark: :heavy_check_mark: :ship:
    * RTI OSRF Debs
      * **Steve (in process)** [install fails if opensplice isn't installed](https://github.com/ros2/rmw_opensplice/issues/79)
    * RTI Trial Binaries
      * **Tully** :heavy_check_mark:
  * OSX
* Demos:
  * real-time
    * Linux
      * **Tully**
        * From binary: :heavy_check_mark: (Unable to get zero pagefaults using RC2)
        * From source: :heavy_check_mark:
  * intra-process
    * Linux
      * **Brian**
        * From binary: :heavy_check_mark: 
        * From source: :heavy_check_mark: 
    * OSX
      * **Brian**
        * From binary: :heavy_check_mark: 
        * From source: :heavy_check_mark: 
    * Windows
  * embedded (at least one person should grab hardware from Morgan and try the whole thing)
    * Linux
      * **Esteve**
        * OpenSplice talker - FreeRTPS listener :heavy_check_mark: 
        * OpenSplice listener - FreeRTPS talker :cry:
        * OpenSplice listener (modified to use best effort) - FreeRTPS talker :heavy_check_mark:  
    * no-OS
      * **Esteve**
        * OpenSplice talker - FreeRTPS listener :heavy_check_mark: 
        * OpenSplice listener - FreeRTPS talker :cry:
        * OpenSplice listener (modified to use best effort) - FreeRTPS talker :heavy_check_mark:  
  * ROS 1 bridge
    * OSX
      * **Brian** (in process)
    * Linux
      * **Tully** :heavy_check_mark: (binary install, except gui)
      * **Steve** :heavy_check_mark: (binary install, except gui)
  * QoS
    * one remaining TODO in the tutorial: "The code explained"
    * Linux
      * **Dirk** :heavy_check_mark: 
    * OSX
      * **William**
    * Windows
* Stuff not covered by the demos:
  * services - **Esteve** :heavy_check_mark: 
  * parameters - **Esteve** :heavy_check_mark: 
  * documentation entry point
    * **Brian** :heavy_check_mark: 