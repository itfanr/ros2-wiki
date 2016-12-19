# Installing ROS 2 on Windows

This page explains how to install ROS 2 on Windows from a pre-built binary package.

## System requirements

We support Windows 8.1. Windows 10 support is experimental.

## Installing prerequisites

### Install Chocolatey

Chocolatey is a package manager for Windows, install it by follow their installation instructions:

https://chocolatey.org/

You'll use Chocolatey to install some other developer tools.

### Install Python

Open a Command Prompt and type the following to install Python via Chocolatey:

```
> choco install -y python
```

### Install Visual Studio Community 2015

If you already have a paid version of Visual Studio 2015 (Professional, Enterprise), skip this step.

Microsoft provides a free of charge version of Visual Studio 2015, named Community, which can be used to build applications that use ROS2:

https://www.visualstudio.com/products/visual-studio-community-vs

Make sure that the Visual C++ features are installed. First choose 'Custom installation':

![Custom installation](http://i.imgur.com/tUcOMOA.png)

Next check Visual C++:

![Visual C++](http://i.imgur.com/yWVEUkm.png)

Ensure that the correct features will be installed:

![Summary](http://i.imgur.com/VxdbA7G.png)

### Getting a DDS Vendor

You'll also need a DDS Vendor available for ROS to work with.
The binary package only supports for PrismTech's OpenSplice; to use another DDS vendor, [build from source](Windows-Development-Setup).

#### OpenSplice

Download a patched version from our OpenSplice fork on GitHub:

https://github.com/osrf/opensplice/releases/download/6.4.0-0/opensplice-Win64VS2013-with-VS2015-patch.zip

Once downloaded you can extract it to `C:\dev\opensplice`

#### eProsima Fast-RTPS & Boost

Fast-RTPS is an alternative vendor for which a binary ROS version exists. Fast-RTPS requires boost as a dependency. To install it grab the appropriate installer from SourceForge [here](http://sourceforge.net/projects/boost/files/boost-binaries/1.61.0/). We test with [this](http://downloads.sourceforge.net/project/boost/boost-binaries/1.61.0/boost_1_61_0-msvc-14.0-64.exe) binary.

The installer will by default install itself into `C:\local`. You will then need to add the following system environment variables for ROS to find the libraries.

`PATH=C:\local\boost_1_61_0\lib64-msvc-14.0`

### Install OpenCV

Some of the examples require OpenCV to be installed. You can download a precompiled version of OpenCV from:

https://github.com/ros2/ros2/releases/download/release-alpha1/opencv-2.4.11-win-vs2015-x64.zip

Since you are using a precompiled ROS version, we have to tell it where to find the OpenCV libraries. Assuming you were extracting OpenCV to `c:\dev\` you have to extend the `PATH` variable to `c:\dev\opencv-2.4.11-win
-vs2015-x64\build\x64\vc14\bin`

## Downloading ROS 2

* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for Windows, e.g., `ros2-package-windows.zip`.
  * Note: there may be more than one binary download option which might cause the file name to differ.
* Unpack the zip file somewhere (we'll assume `C:\dev\ros2`).

## Try some examples

Before running an example, you need to `source` the ROS 2 setup file. Start a command shell, then run a talker:

```
> call C:\dev\ros2\local_setup.bat
> talker
```

Start another command shell and run a listener:

````
> call C:\dev\ros2\local_setup.bat
> listener
```

You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!


### Troubleshooting
* If at one point your example would not start because of missing dll's, please verify that all libraries from external dependencies such as boost or opencv are located inside your `PATH` variable.
* If you forget to call the `local_setup.bat` file from your terminal, the demo programs will most likely crash immediately.

