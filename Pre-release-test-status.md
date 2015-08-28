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
  * OSX
      * **William**
* From-source installation (best to start with a clean VM) using https://gist.github.com/dirk-thomas/25906bd5af20e64d311c:
  * Windows
  * Linux
    * OpenSplice OSRF Debs - **Morgan** :heavy_check_mark:
      * **Tully**
    * OpenSplice binaries 
      * **Morgan (in process)**
    * RTI OSRF Debs
    * RTI Trial Binaries
      * **Tully**
  * OSX
* Demos:
  * real-time
    * Linux
      * **Tully**
  * intra-process
    * Linux
      * **Brian**
        * From binary: :heavy_check_mark: 
    * OSX
      * **Brian**
        * From binary: works, except for the opencv-dependent demo, which gives this error:
                
                $ image_pipeline_all_in_one
                dyld: Library not loaded: /usr/local/Cellar/libpng/1.6.17/lib/libpng16.16.dylib
                  Referenced from: /usr/local/lib/libopencv_highgui.2.4.dylib
                  Reason: image not found
                Trace/BPT trap: 5
                Brians-Air:ros2_install gerkey$ ls /usr/local/Cellar/libpng/1.6.17/lib
                ls: /usr/local/Cellar/libpng/1.6.17/lib: No such file or directory
                Brians-Air:ros2_install gerkey$ ls /usr/local/Cellar/libpng/1.6.1
                1.6.10/ 1.6.12/ 1.6.13/ 1.6.15/ 1.6.18/ 
    * Windows
  * embedded (at least one person should grab hardware from Morgan and try the whole thing)
    * Linux
      * **Esteve**
    * no-OS
      * **Esteve**
  * ROS 1 bridge
    * OSX
      * **Brian**
    * Linux
  * QoS
    * Linux
      * **Dirk**
    * OSX
      * **William**
    * Windows
* Stuff not covered by the demos:
  * services - **Esteve** :heavy_check_mark: 
  * parameters - **Esteve** :heavy_check_mark: 
  * documentation entry point
    * **Brian** :heavy_check_mark: 