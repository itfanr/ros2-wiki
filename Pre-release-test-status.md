Things to be tested are listed below.
Add your name to claim an item and strike it through / check it off when it's done.
Feel free to add more items here.

* Review pending pull requests (not testing, but we need to get everything merged)
  * List PRs here that need review

* Create up-to-date packages from a known state (e.g. create .repos file with hashed as gist)
* From-binary installation (best to start with a clean VM) using https://github.com/ros2/ros2/releases/tag/pre-alpha-rc1:
  * Windows
      * **Esteve**
  * Linux
    * **Tully**
    * **Steve** installed with docker `osrf/ros:indigo-desktop` image. :heavy_check_mark:
  * OSX
      * **William**
* From-source installation (best to start with a clean VM) using https://gist.github.com/dirk-thomas/25906bd5af20e64d311c:
  * Windows
  * Linux
    * OpenSplice OSRF Debs - **Morgan** :heavy_check_mark:
      * **Tully** :heavy_check_mark:
    * OpenSplice binaries 
      * **Morgan (in process)**
    * RTI OSRF Debs
    * RTI Trial Binaries
      * **Tully** :heavy_check_mark:
  * OSX
* Demos:
  * real-time
    * Linux
      * **Tully**
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
    * Linux
      * **Brian**
      * **Tully** :heavy_check_mark: (binary install, except gui)
      * **Steve** :heavy_check_mark: (binary install, except gui)
  * QoS
    * Linux
      * **Dirk**
      * two remaining TODOs in the tutorial
    * OSX
      * **William**
    * Windows
* Stuff not covered by the demos:
  * services - **Esteve** :heavy_check_mark: 
  * parameters - **Esteve** :heavy_check_mark: 
  * documentation entry point
    * **Brian** :heavy_check_mark: 