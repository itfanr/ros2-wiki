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
Anytime you are doing anything you should use this command prompt so that, for example, CMake can find the C compiler and commands like `msbuild` are on the `PATH`.

It is probably worth right clicking on the shortcut, selecting properties, then advanced and checking the run as administrator box so that it always opens as administrator.
Then you can right click it again and say pin to taskbar so it's easy to get to.

Next we can actually install some dependencies.

First install git:

```
> choco install -y git
```
If you are on Windows 8, you will need to append the git bin folder `C:\Program Files\Git\bin` to the PATH (this is not necessary on Windows 10).

Then Python 3.5 or higher:

```
> choco install -y python
```

Then CMake 3.5 or higher

```
> choco install -y cmake
```

You will need to append the CMake bin folder `C:\Program Files\CMake\bin` to the PATH (you can do this by clicking the Windows icon, typing "Environment Variables", then clicking on "Edit the system environment variables".  In the resulting dialog, click "Environment Variables", the click "Path" on the bottom pane, then click "Edit" and add the path).

You may need to close the cmd prompt and open a new one, but at this point you should be able to run `git`, `python`, and `cmake`.

### Installing Boost

FastRTPS requires boost as a dependency. To install it grab the appropriate installer from SourceForge [here](http://sourceforge.net/projects/boost/files/boost-binaries/1.61.0/) We test with [this binary](http://downloads.sourceforge.net/project/boost/boost-binaries/1.61.0/boost_1_61_0-msvc-14.0-64.exe).

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

Now we are ready to install some our tools that we use to help in developing ROS 2.

Let's start with `vcstool`:

```
> pip install vcstool
```

You can test it out by just running `vcs` (you should be able to do this in the same cmd prompt).

Also, you should install `curl`:

```
> choco install -y curl
```

### Install dependencies
In order to install the packages for the robot state publisher correctly, you need to install a few external dependencies. In order to ease the manual installation process, we provide the necessary chocolatey packages.

Please download the two packages for tinyxml and eigen from [this](https://github.com/ros2/choco-packages/releases) github repository. 
Once the two packages are downloaded, open an administrative shell and execute the following two commands:

```
> choco install -y -s <PATH\TO\DOWNLOADS\> eigen tinyxml-usestl
```

Please replace `<PATH\TO\DOWNLOADS>` with the folder you downloaded the two packages in.
 
### Getting the Source Code

Now that we have the development tools we can get the ROS 2 source code.

First setup a development folder, I use `C:\dev\ros2`:

```
> md \dev\ros2\src
> cd \dev\ros2
```

Get the `ros2.repos` file which defines the repositories to clone from:

```
# CMD
> curl -sk https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -o ros2.repos

# PowerShell
> curl https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -o ros2.repos
```

This will get the code for the latest ROS 2 release. If you want the code from a particular release or from the development branches, see [this page](Maintaining-a-Source-Checkout).

Next you can use `vcs` to import the repositories listed in the `ros2.repos` file:

```
# CMD
> vcs import src < ros2.repos

# PowerShell
> vcs import --input ros2.repos src
```

### Getting a DDS Vendor

You'll also need a DDS Vendor available for ROS to build against.
There is currently support for RTI's Connext DDS, and eProsima FastRTPS.
The source distribution of ROS 2 includes FastRTPS, so it will always build unless explicitly ignored.

If you would like to also build against RTI Connext, you will need to first visit the RTI website and obtain a license (evaluation or purchased) for RTI Connext DDS, and then something like the following command in your shell before building ROS 2:

```
call "C:\Program Files\rti_connext_dds-5.2.3\resource\scripts\rtisetenv_x64Win64VS2015.bat"
```
where the exact paths may need to be slightly altered depending on where you selected to install RTI Connext DDS. The path above is the current default path as of version 5.2.3, but will change as the version numbers increment in the future.

Otherwise, ROS 2 will default to using eProsima FastRTPS as the middleware.

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
You will need to add C:\Program Files\Cppcheck to the PATH:

### Install a binary distribution of OpenCV compatible with Visual Studio 2015

You can download a precompiled version of OpenCV from:

https://github.com/ros2/ros2/releases/download/release-alpha1/opencv-2.4.11-win-vs2015-x64.zip

Assuming you unpacked it to `C:\opencv`, type the following on a Command Prompt (requires Admin privileges):

```
setx -m OpenCV_DIR C:\opencv\build
```
You will also need to add the OpenCV bin directory (in this case `C:\opencv\build\x64\vc14\bin`) to the `PATH`.

You will need to open a new command prompt for this environment variable to take effect

### Building the ROS 2 Code

FastRTPS is bundled with the ROS 2 source and will always be built unless you put an `AMENT_IGNORE` file in the `src\eProsima` folder.

To build the `\dev\ros2` folder tree:

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

### Maintaining your Source Checkout
For information on how to keep your source checkout up-to-date, see [Maintaining a Source Checkout](Maintaining-a-Source-Checkout).

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
> choco install -y svn hg
```
* You'll need to quit and restart the command prompt after installing the above.
* Install the Python 3.6.0 source from the tarball [here](https://www.python.org/ftp/python/3.6.0/Python-3.6.0.tgz). To keep these instructions concise, please extract it to C:\dev\Python-3.6.0
* Now, build the Python source in debug mode from a Visual Studio command prompt (it may need to be in Administrator mode; can't remember right now...)
```
> cd C:\dev\Python-3.6.0\PCbuild
> get_externals.bat
> build.bat -p x64 -d
```
* Finally, copy the build products into the Python36 installation directories, next to the Release-mode Python executable and DLL's:
```
> cd C:\dev\Python-3.6.0\PCbuild\amd64
> copy python_d.exe C:\Python36
> copy python36_d.dll C:\Python36
> copy python3_d.dll C:\Python36
> copy python36_d.lib C:\Python36\libs
> copy python3_d.lib C:\Python36\libs
> for %I in (*_d.pyd) do copy %I C:\Python36\DLLs
```
* Now, from a fresh command prompt, make sure that `python_d` works:
```
> python_d
> import _ctypes
```
* To create executables python scripts(.exe), python_d should be used to invoke ament_tools
```
> python_d src\ament\ament_tools\scripts\ament.py build 
```
* Hooray, you're done!