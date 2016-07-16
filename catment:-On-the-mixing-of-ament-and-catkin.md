# catment: On the mixing of ament and catkin

**All that follows is experimental and speculative.**

* [Background](#background)
* [Postulates](#postulates)
* [Use cases, with experimental implementations](#use-cases-with-experimental-implementations)

## Background

There once was a thing called `rosbuild`.
Then came a thing called `catkin`, which largely replaced `rosbuild`.
Recently introduced is a thing called `ament`, which may one day replace `catkin`.

All three tools can be considered "meta-build systems."
They sit atop other build systems (e.g., CMake, Python setuptools) and provide extra functionality that's intended to make those build systems easier to use, especially when managing dependencies across multiple packages and when building multiple packages in a single workspace.

Each of these meta-build systems does two things:

1. Add API to the underlying build system (e.g,. CMake) that can be used to simplify common tasks (e.g., supplying all the flags exported by depended-upon packages when building an executable).
There are usually hooks to allow injection of extra API by packages outside of the core meta-build system.

 * `rosbuild`: `mk/cmake.mk`, `rosbuild_init()`, `rosbuild_add_executable()`, etc.
 * `catkin`: `catkin_package()`, `catkin_install_python()`, etc.
 * `ament`: `ament_target_dependencies()`, `ament_export_dependencies()`, `ament_package()`, etc.

1. Provide a tool that can be used to iterate in dependency order over a workspace full of packages, building and perhaps installing each one.

  * `rosbuild`: `rosmake`
  * `catkin`: `catkin build`, `catkin_make`, `catkin_make_isolated`, etc.
  * `ament`: `ament build`

The common thread that ties all of these systems together is the division of the code into **packages**, with each package containing a manifest file (`manifest.xml` or `package.xml`).
This manifest is required (with some exceptions) for both parts of the meta-build system (API and building tool) to function.

## Postulates

1. **While we usually consider the two aspects of a meta-build system to be coupled, they needn't be.**
The API used inside a package and the tool that iterates over the packages can be considered largely independent, with the package manifest forming the interface between them.
There's no reason in principle why, for example, `rosmake` couldn't be modified to iterate over a workspace filled with `catkin` packages, stepping into them in dependency order and doing the usual `mkdir build; cd build; cmake ..; make install` routine for each one (with appropriate flags passed to `cmake` and `make`).
1. **The effort required to migrate from one meta-build system to another should be minimized.**
The mass migration from `rosbuild` to `catkin` was difficult and remains a sore point for many in the community.
While it's reasonable to ask developers to make changes in exchange for getting access to new functionality, the changes that are required should be as small as possible without sacrificing the effectiveness of the new system.
This is especially true when the old system is in widespread use.

  1. Corollary: **Migration to a new meta-build system should not be required without a very good reason.**
  If a developer doesn't want the functionality offered by the new system, then she shouldn't be coerced into migrating from the old system unless there's something irrevocably broken about the old system (e.g., `rosbuild`'s in-source build pattern and lack of an "install" step).

1. **Interoperability is a good thing.**
Whenever possible (not all combinations will be practical), developers should be able to mix and match meta-build systems, including mixing their different aspects (i.e., use the building tool from one system and the API from another).
Such mixing and matching is especially important when developers want to combine a large existing code base using one meta-build system (e.g., ROS with `catkin`) with new libraries and tools offered by a code base using another meta-build system (e.g., ROS2 with `ament`).
Ideally that kind of combination can be done without requiring changes to the API used by either code base and without telling the developer which builder tool to use.

## Use cases, with experimental implementations

### Adding ROS packages to a ROS2 workspace and building with `ament build`

Let's say that you want to add some existing ROS packages to your ROS2 workspace and don't want to migrate the ROS packages from `catkin` to `ament` (or vice versa). Here are two patches that let you do that:

* [ament_package](https://github.com/ament/ament_package/compare/catkin?expand=1):
Add support for format 1 package manifests, instead of requiring format 2.
This change isn't strictly related to `catkin` vs. `ament`, because format 2 has been around for a while and `catkin` supports it, so developers could already update their manifests to format 2.
But there's a ton of ROS code out there that uses format 1, so we should support it.
This implementation could be improved, e.g., by reasoning over the various flavors of depend tags and how they differ between formats 1 and 2.
* [ament_tools](https://github.com/ament/ament_tools/compare/catkin?expand=1):
Add a new `catkin` build type to `ament`.
This implementation just treats `catkin` packages the same as plain `cmake` packages, which seems to work fine.
It could be made more sophisticated.

Example usage:

1. Get the ROS2 code as usual, using the branches mentioned above.
1. Add to your workspace some `catkin` ROS packages, ensuring that all of their dependencies are satisfied (either also present in the workspace or installed elsewhere with appropriate setup shell files sourced).
1. Build as usual (e.g., `./src/ament/ament_tools/scripts/ament.by build`).

Voila: your existing code isn't suddenly broken just because there's a new builder tool in use.

#### Variation: Building ROS packages with `ament build`

Let's say that you love the new `ament` tool and want to use it to build your existing ROS packages that use `catkin` internally.
Here's an example of how to do that, by doing a minimal installation of `ament` and then using it to build a workspace full of ROS `catkin` packages:

```
git clone https://github.com/osrf/osrf_pycommon.git
cd osrf_pycommon
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/ament ..
make install
git clone https://github.com/ament/ament_package.git
cd ament_package
git checkout catkin
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/ament ..
make install
git clone https://github.com/ament/ament_tools.git
cd ament_tools
git checkout catkin
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/ament ..
make install
. /tmp/ament/setup.bash
cd ~/ros_catkin_ws
ament build
```

Voila: you used the `ament` build tool to build your `catkin` packages, without having to migrate them.

### Building ROS2 packages with `catkin_make_isolated`

Let's say that you're already familiar with ROS and `catkin` and that you're excited to try ROS2, but that you're not in the mood to learn about `ament`.
You'd rather stick to what you know, such as using `catkin_make_isolated` to build everything.
Here is a patch that allows you to do that:

* [catkin](https://github.com/ros/catkin/compare/ament?expand=1):
Add support for packages that declare themselves to have a build type of `ament_*`.
This implementation calls out to `ament` to build each such package.
While `ament_cmake` packages can be treated as plain `cmake` packages (as we did when adding `catkin` support to `ament`), `ament_python` packages require some gnarly invocations of Python.
Instead of trying to replicate that logic in `catkin`, it's easier to just let `ament` handle it.

Example usage, installing the modified version of catkin somewhere, then using it to build the usual ROS2 workspace:

```
git clone https://github.com/ros/catkin.git
cd catkin
git checkout ament
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/catkin ..
make install
. /tmp/catkin/setup.bash
cd ~/ros2_ws
catkin_make_isolated
```

Voila: you've built ROS2 using the tools you're familiar with.
*Caveat: I'm currently getting a build failure in `builtin_interfaces` due to an apparent dependency problem related to `rosidl_generator_c`; investigation required.*

### Combining all of ROS and ROS2 in one workspace and building it (TODO)

This step will require sorting out some things, including at least:

* Package name conflicts.
We currently have ROS2 versions of ROS message packages, as well as some stuff in `geometry2`.
Either the functionality needs to be merged into one package that can support both systems, or the new versions need different names.
* Message generation.
ROS and ROS2 have different message generation steps, the output of which might or not might conflict.
Something sort of sophisticated needs to be done to allow generation of all the right artifacts from a single message package (or, as indicated above, the new message packages need different name).

### Using `bloom` to release `ament` packages (TODO)

It seems like `bloom` ought be able to release packages that use the `ament` CMake API, and that the resulting releases should be able to be built on the farm.
We can make changes to `bloom` and `ros_buildfarm` as needed to enable this use case.