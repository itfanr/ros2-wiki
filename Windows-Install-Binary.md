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

## Downloading ROS 2

* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for Windows, e.g., `ros2-package-windows.zip`.
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
