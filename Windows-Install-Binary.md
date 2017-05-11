# Installing ROS 2 on Windows

This page explains how to install ROS 2 on Windows from a pre-built binary package.

## System requirements
Until beta-1 we supported Windows 8.1 and Windows 10.
As of beta-2 only Windows 10 is supported.

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

The binary package bundles eProsima FastRTPS as the middleware. To use another DDS vendor, you will need to [build from source](Windows-Development-Setup).

#### eProsima FastRTPS & Boost (only for beta-1 and older releases)

FastRTPS requires boost as a dependency. To install it, grab the appropriate installer from SourceForge [here](http://sourceforge.net/projects/boost/files/boost-binaries/1.61.0/). We test with [this](http://downloads.sourceforge.net/project/boost/boost-binaries/1.61.0/boost_1_61_0-msvc-14.0-64.exe) binary.

The installer will install itself by default into `C:\local`. You will then need to add the following system environment variables for ROS to find the libraries.

`PATH=C:\local\boost_1_61_0\lib64-msvc-14.0`

### Install OpenCV

Some of the examples require OpenCV to be installed. You can download a precompiled version of OpenCV from:

https://github.com/ros2/ros2/releases/download/release-alpha1/opencv-2.4.11-win-vs2015-x64.zip

Since you are using a precompiled ROS version, we have to tell it where to find the OpenCV libraries. Assuming you were extracting OpenCV to `c:\dev\` you have to extend the `PATH` variable to `c:\dev\opencv-2.4.11-win-vs2015-x64\build\x64\vc14\bin`

### Install dependencies
There are a few dependencies not available in the Chocolatey package database. In order to ease the manual installation process, we provide the necessary Chocolatey packages.

Please download these packages from [this](https://github.com/ros2/choco-packages/releases/latest) GitHub repository. 
 * asio.1.10.6.nupkg
 * eigen-3.3.3.nupkg
 * tinyxml-usestl.2.6.2.nupkg
 * tinyxml2.4.1.0.nupkg

Once the these packages are downloaded, open an administrative shell and execute the following command:

```
> choco install -y -s <PATH\TO\DOWNLOADS\> asio eigen tinyxml-usestl tinyxml2
```

Please replace `<PATH\TO\DOWNLOADS>` with the folder you downloaded the packages to.
 
## Downloading ROS 2

* Go the releases page: https://github.com/ros2/ros2/releases
* Download the latest package for Windows, e.g., `ros2-package-windows-AMD64.zip`.
  * Note: there may be more than one binary download option which might cause the file name to differ.
* Unpack the zip file somewhere (we'll assume `C:\dev\ros2`).

## Try some examples

Before running an example, you need to `source` the ROS 2 setup file. Start a command shell, then run a talker:

```
> call C:\dev\ros2\local_setup.bat
> talker
```

Start another command shell and run a listener:

```
> call C:\dev\ros2\local_setup.bat
> listener
```

You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
Hooray!


### Troubleshooting
* If at one point your example would not start because of missing dll's, please verify that all libraries from external dependencies such as opencv are located inside your `PATH` variable.
* If you forget to call the `local_setup.bat` file from your terminal, the demo programs will most likely crash immediately.
