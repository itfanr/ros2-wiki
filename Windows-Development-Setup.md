# Building ROS 2 on Windows

This guide is about how to setup a development environment for ROS2 on Windows.

## Prerequisites
Until beta-1 we supported Windows 8.1 and Windows 10.
As of beta-2 only Windows 10 is supported.

### Installing Visual Studio

First you'll need Visual Studio 14.0 2015 Update 3 installed. With Visual C++ selected during the installation (It is not selected by default in Community Edition)

We are testing with VS2015 Update 3. You can get it from here: https://www.visualstudio.com/vs/older-downloads/ with an MSDN account or Free Dev Essential account.

2017 is newer but is known to have some issues that have not been resolved.

Windows 8.1:
Press the windows key and type `visual studio tools`, which should find a folder called "Visual Studio Tools". Or type `cmd`.
In that folder there will be a shortcut called "VS2015 x64 Native Tools Command Prompt".
Right click on this shortcut and select "Run as administrator".

Windows 10:
Press the windows key and type `apps: vs2015 x64 native tools command prompt` then right click on the app with the same name and select "Run as administrator".

This will open a cmd prompt with the Visual Studio tools on the `PATH` and running as administrator's permissions.
Anytime you are doing anything you should use this command prompt so that, for example, CMake can find the C compiler and commands like `msbuild` are on the `PATH`.

It is probably worth right clicking on the shortcut, selecting properties, then advanced and checking the run as administrator box so that it always opens as administrator.
Then you can right click it again and say pin to taskbar so it's easy to get to.


### Install Chocolatey

Chocolatey is a package manager for Windows, install it by following their installation instructions:

https://chocolatey.org/

You'll use Chocolatey to install some other developer tools.

### Installing some dependencies

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

Add ```<PATH\TO\PYTHON>\Scripts``` to your PATH if choco's installer has not done it for you already.  

Then CMake 3.5 or higher

```
> choco install -y cmake
```

You will need to append the CMake bin folder `C:\Program Files\CMake\bin` to the PATH (you can do this by clicking the Windows icon, typing "Environment Variables", then clicking on "Edit the system environment variables".
In the resulting dialog, click "Environment Variables", the click "Path" on the bottom pane, then click "Edit" and add the path).

Then install `patch`:

```
> choco install -y patch
```

You may need to close the cmd prompt and open a new one, but at this point you should be able to run `git`, `python`, `cmake`, and `patch.exe --version`.

### Installing Boost (only for beta-1 and older)

Up until beta-1, FastRTPS required boost as a dependency.
To install it grab the appropriate installer from SourceForge [here](http://sourceforge.net/projects/boost/files/boost-binaries/1.61.0/) We test with [this binary](http://downloads.sourceforge.net/project/boost/boost-binaries/1.61.0/boost_1_61_0-msvc-14.0-64.exe).

The installer will by default install itself into `C:\local` 

You will then need to add the following system environment variables.
```
BOOST_ROOT=C:\local\boost_1_61_0
BOOST_LIBRARYDIR=C:\local\boost_1_61_0\lib64-msvc-14.0
```

You will also need to make sure the Boost DLLs are listed in the PATH environment variable.
For example,

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

In order to install the packages for the robot state publisher correctly, you need to install a few external dependencies.
In order to ease the manual installation process, we provide the necessary chocolatey packages.

Please download these packages from [this](https://github.com/ros2/choco-packages/releases/latest) GitHub repository. 
 * asio.1.10.6.nupkg
 * eigen-3.3.3.nupkg
 * tinyxml-usestl.2.6.2.nupkg
 * tinyxml2.4.1.0.nupkg

Once these packages are downloaded, open an administrative shell and execute the following command:

```
> choco install -y -s <PATH\TO\DOWNLOADS\> asio eigen tinyxml-usestl tinyxml2
```

Please replace `<PATH\TO\DOWNLOADS>` with the folder you downloaded the packages to.

Next install the latest version of `setuptools` and `pip`:

```
> pip install -U setuptools pip
```

Then you can continue installing other Python dependencies:

```
> pip install EmPy pyparsing pyyaml
```

Next install testing tools like `Nose` and others:

```
> pip install nose coverage mock pytest pytest-cov pytest-runner
```

Next install linters and checkers like `flake8` and others:

```
> pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pep8 pydocstyle pyflakes
```

Also install `trollius` now so that it uses the wheel rather than trying to install from source:

```
> pip install trollius
```

Next install cppcheck:

```
> choco install -y cppcheck
```

You will need to add C:\Program Files\Cppcheck to the PATH.

### Install a binary distribution of OpenCV

You can download a precompiled version of OpenCV from:

- Visual Studio 2015:
  - https://github.com/ros2/ros2/releases/download/release-beta2/opencv-2.4.13.2-vc14.VS2015.zip
- Visual Studio 2017:
  - https://github.com/ros2/ros2/releases/download/release-beta1/opencv-2.4.13.2-vc15update.zip

Assuming you unpacked it to `C:\opencv`, type the following on a Command Prompt (requires Admin privileges):

```
setx -m OpenCV_DIR C:\opencv
```

You will also need to add the OpenCV bin directory (in this case `C:\opencv\x64\vc14\bin` for VS2015 or `C:\opencv\x64\vc15\bin` for VS2017) to the `PATH`.

You will need to open a new command prompt for this environment variable to take effect.

### Install OpenSSL

Download an OpenSSL installer from [this page](https://slproweb.com/products/Win32OpenSSL.html). Scroll to the bottom of and download *Win64 OpenSSL v1.0.2*. Don't download the Win32 or Light versions.

Run the installer with default parameters. Then, define environment variables (the following commands assume you used the default installation directory):

- `setx -m OPENSSL_CONF C:\OpenSSL-Win64\bin\openssl.cfg`
- Add `C:\OpenSSL-Win64\bin\` to your PATH

### Install Qt5

This section is only required if you are building rviz, but it comes with our default set of sources, so if you don't know, then assume you are building it.

First get the installer from Qt's website:

https://www.qt.io/download

Select the Open Source version and then the `Qt Online Installer for Windows`.

Run the installer and install Qt5.
We recommend you install it to the default location of `C:\Qt`, but if you choose somewhere else, make sure to update the paths below accordingly.
When selecting what to install, the only thing you absolutely need is the `MSVC 2015 64-bit` option under the `Qt` -> `Qt 5.10.0` tree.
We're using `5.10.0` as of the writing of this document and that's what we recommend since that's all we test on Windows, but later version will probably work too.
You may also want to go ahead and install the `MSVC 2017 64-bit` too if you'd like to use Visual Studio 2017 (we have only experimental support for this right now).
After that, the default settings are fine.

Finally, set the `Qt5_DIR` environment variable in the `cmd.exe` where you intend to build so that CMake can find it:

```
> set Qt5_DIR=C:\Qt\5.10.0\msvc2015_64
: You could set it permanently with `setx -m Qt5_DIR C:\Qt\5.10.0\msvc2015_64` instead, but that requires Administrator.
```

Note, this path might change based on which MSVC version you're using or if you installed it to a different directory.

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

> Note: if you want to get all of the latest bug fixes then you can try the "tip" of development by replacing `release-latest` in the URL above with `master`. The `release-latest` is preferred by default because it goes through more rigorous testing on release than changes to master do. See also [Maintaining a Source Checkout](https://github.com/ros2/ros2/wiki/Maintaining-a-Source-Checkout).

Next you can use `vcs` to import the repositories listed in the `ros2.repos` file:

```
# CMD
> vcs import src < ros2.repos

# PowerShell
> vcs import --input ros2.repos src
```

### Getting a DDS Vendor

You'll also need a DDS Vendor available for ROS to build against.
There is currently support for eProsima FastRTPS, Adlink's OpenSplice, and RTI's Connext DDS.
The source distribution of ROS 2 includes FastRTPS, so it will always build unless explicitly ignored.

#### Adlink OpenSplice

If you would like to also build against OpenSplice, you will need to first download the latest version of [OpenSplice 6.7.171127](https://github.com/ADLINK-IST/opensplice/releases/tag/OSPL_V6_7_171127OSS_RELEASE) (we require at least version 6.7.170912).
Then run something like the following command before building ROS 2, to set up the OpenSplice environment:

```
call "C:\opensplice67\HDE\x86_64.win64\release.bat"
```

where the exact paths may need to be slightly altered depending on where you selected to install OpenSplice.

#### RTI Connext 5.3

If you would like to also build against RTI Connext, you will need to first visit the RTI website and obtain a license (evaluation or purchased) for RTI Connext DDS as well as the installer via their [downloads page](https://www.rti.com/downloads).
After installing, use the RTI Launcher to load your license file.
Then before building ROS 2, set up the Connext environment:

```
call "C:\Program Files\rti_connext_dds-5.3.0\resource\scripts\rtisetenv_x64Win64VS2015.bat"
```

Note that this path might need to be slightly altered depending on where you selected to install RTI Connext DDS.
The path above is the current default path as of version 5.3.0, but will change as the version numbers increment in the future.

If you don't install any additional DDS vendors, ROS 2 will default to using eProsima's Fast-RTPS as the middleware.

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

Note, if you are doing a debug build use `python_d` instead of `python`.
See: https://github.com/ros2/ros2/wiki/Windows-Development-Setup#extra-stuff-for-debug-mode for more info on running Python code in debug builds on Windows.

### Testing and Running

Note that the first time you run any executable you will have to allow access to the network through a Windows Firewall popup.

You can run the tests using this command:

```
> python src\ament\ament_tools\scripts\ament.py test
```

Afterwards you can get a summary of the tests using this command:

```
> python src\ament\ament_tools\scripts\ament.py test_results
```

To run the examples, first open a clean new `cmd.exe` and set up the workspace.
This is done by sourcing the `local_setup.bat` file, which will automatically set up the environment for any DDS vendors that support was built for.
Then execute the examples, e.g.:

```
> call install\local_setup.bat
> ros2 run demo_nodes_py talker
```

In a separate shell you can do the same, but instead run the `listener`:

```
> call install\local_setup.bat
> ros2 run demo_nodes_py listener
```

For more explanations see the [Python Programming](Python-Programming) demo or [other tutorials](https://github.com/ros2/ros2/wiki/Tutorials)

Note: it is not recommended to build in the same cmd prompt that you've sourced the `local_setup.bat`.

### Alternative DDS Sources

The demos will attempt to build against any detected DDS vendor.
The only bundled vendor is eProsima's Fast RTPS, which is included in the default set of sources for ROS 2.0.
To build for other vendors, make sure that your chosen DDS vendor(s) are exposed in your environment when you run the build.
If you would like to change which vendor is being used see: [Working with Multiple RMW Implementations](https://github.com/ros2/ros2/wiki/Working-with-multiple-RMW-implementations)

## Troubleshooting

### CMake error setting modification time

If you run into the CMake error `file INSTALL cannot set modification time on ...` when installing files it is likely that an anti virus software or Windows Defender are interfering with the build. E.g. for Windows Defender you can list the workspace location to be excluded to prevent it from scanning those files.

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
* Get and extract the Python 3.6.4 source from the `tgz`:
  * https://www.python.org/ftp/python/3.6.4/Python-3.6.4.tgz
  * To keep these instructions concise, please extract it to `C:\dev\Python-3.6.4`
* Next you'll have to patch the python build files for Windows:
  * https://bugs.python.org/issue32423
  * https://github.com/isuruf/cpython/commit/9432a2c7f63b3bb55e8066e91eade81321154476
    * use this diff as a guide on what to change
* Now, build the Python source in debug mode from a Visual Studio command prompt:

```
> cd C:\dev\Python-3.6.4\PCbuild
> get_externals.bat
> build.bat -p x64 -d
```

* Finally, copy the build products into the Python36 installation directories, next to the Release-mode Python executable and DLL's:

```
> cd C:\dev\Python-3.6.4\PCbuild\amd64
> copy python_d.exe C:\Python36 /Y
> copy python36_d.dll C:\Python36 /Y
> copy python3_d.dll C:\Python36 /Y
> copy python36_d.lib C:\Python36\libs /Y
> copy python3_d.lib C:\Python36\libs /Y
> for %I in (*_d.pyd) do copy %I C:\Python36\DLLs /Y
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
