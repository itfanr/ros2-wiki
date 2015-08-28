# Installing ROS 2 on Windows
This page explains how to install ROS 2 on Windows from a pre-built binary package.

## System requirements
We support Windows 8.1.

## Installing prerequisites

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
Before running an example, you need to `source` both the OpenSplice setup file and the ROS 2 setup file (TODO: check whether the OpenSplice setup file is really needed)  Start a command shell, then run a talker:
```
> call C:\dev\opensplice\HDE\x86_64.win64\release.bat
> call C:\dev\ros2\local_setup.bat
> talker
```
Start another command shell and run a listener:
````
> call C:\dev\opensplice\HDE\x86_64.win64\release.bat
> call C:\dev\ros2\local_setup.bat
> listener
```
You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!