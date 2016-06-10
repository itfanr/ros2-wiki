# Building ROS 2 on Windows

This guide is about how to setup a development environment for ROS2 on Windows.

## Prerequisites

First you'll need Windows 8.1 or 10 and Visual Studio 14.0 2015 Update 2 installed. With Visual C++ selected during the installation (It is not selected by default in Community Edition)

### Install Chocolatey

Chocolatey is a package manager for Windows, install it by follow their installation instructions:

https://chocolatey.org/

You'll use Chocolatey to install some other developer tools.

### Installing Build Tools

Windows 8.1:
Press the windows key and type `visual studio tools`, which should find a folder called "Visual Studio Tools". Or type `cmd`. In that folder there will be a shortcut called "VS2015 x64 Native Tools Command Prompt".
Right click on this shortcut and select "Run as administrator".

Windows 10:
Press the windows key and type `apps: vs2015 x64 native tools command prompt` then right click on the app with the same name and select "Run as administrator".

This will open a cmd prompt with the Visual Studio tools on the `PATH` and running as administrator's permissions.
Anytime you are doing anything you should use this command prompt so that, for example, CMmake can find the C compiler and commands like `msbuild` are on the `PATH`.

It is probably worth right clicking on the shortcut, selecting properties, then advanced and checking the run as administrator box so that it always opens as administrator.
Then you can right click it again and say pin to taskbar so it's easy to get to.

Next we can actually install some dependencies.

First install git:

```
> choco install -y git
```
You will need to append the CMake bin folder `C:\Program Files\Git\bin` to the PATH. (last seen not needed on Windows 10)

Then Python 3.5 or higher:

```
> choco install -y python
```

Since we require CMake 3.5 (and Chocolatey only provides 3.4.3) you need to download and install the [official CMake package](https://cmake.org/download/).

You will need to append the CMake bin folder `C:\Program Files (x86)\CMake\bin` to the PATH.

You may need to close the cmd prompt and open a new one, but at this point you should be able to run `git`, `python`, and `cmake`.

### Installing Boost

FastRTPS requires boost as a dependency. To install it grab the appropriate installer from SourceForge [here](http://sourceforge.net/projects/boost/files/boost-binaries/1.61.0/) We test with [this binary](http://downloads.sourceforge.net/project/boost/boost-binaries/1.61.0/boost_1_61_0-msvc-14.0-64.exe)

The installer will by default install itself into `C:\local` 

You will then need to add the following system environment variables.
```
BOOST_ROOT=C:\local\boost_1_61_0
BOOST_LIBRARYDIR=C:\local\boost_1_61_0\lib64-msvc-14.0
```

You will also need to make sure the Boost DLLs are listed in the PATH environment variable. For example,

```
PATH=C:\local\boost_1_61_0\lib64-msvc-14.0
```

### Installing Developer Tools

Now we are ready to install some our tools that we use to help in developing ROS 2, but first we need to put Python's `Scripts` folder on the path.
We need to do this so that installed python packages with scripts can be run directly from the prompt.

So, push the windows key and type `environment`, then select "Edit the system environment variables".
Then select the "Environment Variables..." button and edit the entry for the `PATH` under the "system environment variable", adding `C:\ProgramData\chocolatey\lib\python3\tools\Scripts;` to the end.
You might need to open a new cmd prompt for this to take effect.

Now we can actually install those tools, start with `vcstool`:

```
> pip install vcstool
```

You can test it out by just running `vcs` (you should be able to do this in the same cmd prompt).

Also, you should install `curl`:

```
> choco install -y curl
```

### Getting the Source Code

Now that we have the development tools we can get the ROS 2 source code.

First setup a development folder, I use `C:\dev\ros2`:

```
> md \dev\ros2\src
> cd \dev\ros2
```

Get the `ros2.repos` file which defines the repositories to clone from:

```
> curl -sk https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -o ros2.repos
```

Next you can use `vcs` to import the repositories listed in the `ros2.repos` file:

```
> vcs import src < ros2.repos
```

### Getting a DDS Vendor

You'll also need a DDS Vendor available for ROS to build against.
There is support for PrismTech's OpenSplice as well as RTI's Connext DDS.

#### OpenSplice

In this example I'll use PrismTech's OpenSplice.
You can go to their website and login to download their VS2013 binary and then patch it for VS2015 or you can just download the pre-patched version from our OpenSplice fork on GitHub:

https://github.com/osrf/opensplice/releases/download/6.4.0-0/opensplice-Win64VS2013-with-VS2015-patch.zip

Once downloaded you can extract it somewhere, I use `C:\dev\opensplice`, and the continue to the next step of the installation process.

If you want to do it yourself, first download their Windows binary for 64-bit and Visual Studio 2013:

http://www.prismtech.com/dds-community/software-downloads

Once you've downloaded their archive, extract it somewhere, I'll extract mine to `C:\dev\opensplice`.

Also, for now you'll need to patch OpenSplice with this diff:

```diff
--- C:\dev\opensplice\HDE\x86_64.win64\include\include\os_stdlib.h
+++ (clipboard)
@@ -66,7 +66,9 @@

 #define MAXHOSTNAMELEN MAX_HOSTNAME_LEN

+#if _MSC_VER < 1900
 OS_API extern int snprintf(char *s, size_t n, const char *format, ...);
+#endif

 OS_API extern char *optarg;
 OS_API extern int optind, opterr;
```

### Installing a few dependencies

First install the latest version of `setuptools` and `pip`:

```
> pip install -U setuptools pip
```

Then you can continue installing other Python dependencies, install `EmPy`:

```
> pip install EmPy
```

Next install testing tools like `Nose` and others:

```
> pip install nose coverage mock
```

Next install linters and checkers like `flake8` and others:

```
> pip install flake8 pep8 pydocstyle pyflakes
```

Also install `trollius` now so that it uses the wheel rather than trying to install from source:

```
> pip install trollius
```

Next install cppcheck:

```
> choco install -y cppcheck
```

### Install a binary distribution of OpenCV compatible with Visual Studio 2015

You can download a precompiled version of OpenCV from:

https://github.com/ros2/ros2/releases/download/release-alpha1/opencv-2.4.11-win-vs2015-x64.zip

Assuming you unpacked it to `C:\opencv`, type the following on a Command Prompt (requires Admin privileges):

```
setx -m OpenCV_DIR C:\opencv\build
```

### Building the ROS 2 Code

In order to build the ROS 2 Code you must first "source" the `release.bat` file provided by OpenSplice:

```
> call C:\dev\opensplice\HDE\x86_64.win64\release.bat
```

Then simply run this command in the `\dev\ros2` folder:

```
> python src\ament\ament_tools\scripts\ament.py build
```

You can additionally build the tests by adding the `--build-tests` option:

```
> python src\ament\ament_tools\scripts\ament.py build --build-tests
```

### Testing and Running

You can run the tests using this command:

```
> python src\ament\ament_tools\scripts\ament.py test
```

Afterwards you can get a summary of the tests using this command:

```
> python src\ament\ament_tools\scripts\ament.py test_results
```

You can run the built examples by opening a new `cmd.exe` sourcing the `local_setup.bat` file and then executing them directly, for example:

```
> call install\local_setup.bat
> install\bin\talker
```

In a separate shell you can do the same, but instead run the `listener`:

```
> call install\local_setup.bat
> install\bin\listener
```

Note: it is not recommended to build in the same cmd prompt that you've sourced the `local_setup.bat`.

## Troubleshooting

### CMake error setting modification time

If you run into the CMake error `file INSTALL cannot set modification time on ...` when installing files it it likely that an anti virus software or Windows Defender are interfering with the build. E.g. for Windows Defender you can list the workspace location to be excluded to prevent it from scanning those files.

## Extra stuff for Debug mode

If you want to be able to run all the tests in Debug mode, you'll need to install a few more things:

* To be able to extract the Python source tarball, you can use PeaZip:
```
> choco install -y peazip
```
* You'll also need SVN, since some of the Python source-build dependencies are checked out via SVN:
```
> choco install -y svn
```
* Install the Python 3.5.1 source from the tarball [here](https://www.python.org/ftp/python/3.5.1/Python-3.5.1.tgz). To keep these instructions concise, please extract it to C:\dev\Python-3.5.1
* Now, build the Python source in debug mode from a Visual Studio command prompt (it may need to be in Administrator mode; can't remember right now...)
```
> cd C:\dev\Python-3.5.1\PCbuild
> get_externals.bat
> build.bat -p x64 -d
```
* Finally, copy the build products into the chocolatey installation directories, next to the Release-mode Python executable and DLL's:
```
> cd C:\dev\Python-3.5.1\PCbuild\amd64
> copy python_d.exe C:\ProgramData\chocolatey\bin
> copy python35_d.dll C:\ProgramData\chocolatey\bin
> copy python35_d.dll C:\ProgramData\chocolatey\lib\python3\tools
> copy python3_d.dll C:\ProgramData\chocolatey\lib\python3\tools
> copy python35_d.lib C:\ProgramData\chocolatey\lib\python3\tools\libs
> copy python3_d.lib C:\ProgramData\chocolatey\lib\python3\tools\libs
```
* Now, from a fresh command prompt, make sure that `python_d` works (i.e., you enter an interactive session with Python built in debug mode).
* Hooray, you're done!