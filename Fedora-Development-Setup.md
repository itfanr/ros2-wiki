## How to setup the development environment?

First install a bunch of dependencies:

```
$ sudo dnf install cppcheck cmake opencv-devel poco-devel poco-foundation python3-empy python3-devel python3-nose python3-pip python3-pyparsing python3-pytest python3-pytest-cov python3-pytest-runner python3-setuptools python3-yaml tinyxml-devel eigen3-devel python3-pydocstyle python3-pyflakes python3-coverage python3-mock python3-pep8 uncrustify python3-argcomplete python3-flake8 python3-flake8-import-order asio-devel tinyxml2-devel libyaml-devel
```

Then install vcstool from pip:

```
$ pip3 install vcstool
```

With this done, you can follow the rest of the [[instructions|Linux-Development-Setup#get-ros-20-code]] to fetch and build ROS2.