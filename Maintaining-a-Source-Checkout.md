# Maintaining a Source Checkout of ROS 2
If you have installed ROS 2 from source, there may have been changes made to the source code since the time that you checked it out.
To keep your source checkout up to date, you will have to periodically update your `ros2.repos` file, download the latest sources, and rebuild your workspace.

## Update your repository list
Each ROS 2 release includes a `ros2.repos` file that contains the list of repositories and their version for that release.

### Latest release
To download the repository list from the latest ROS 2 release, run:

#### _Linux/OS X_
```
cd ~/ros2_ws
mv -i ros2.repos ros2.repos.old
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
```

#### _Windows_
```
# CMD
> cd \dev\ros2
> curl -sk https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -o ros2.repos

# PowerShell
> cd \dev\ros2
> curl https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -o ros2.repos
```

### Particular release
If you wish to checkout a particular release, you can get its repository list by specifying the name of the release in the url of the following step, e.g. for alpha 7:

#### _Linux/OS X_
```
cd ~/ros2_ws
mv -i ros2.repos ros2.repos.old
wget https://raw.githubusercontent.com/ros2/ros2/release-alpha7/ros2.repos
```

#### _Windows_
```
# CMD
> cd \dev\ros2
> curl -sk https://raw.githubusercontent.com/ros2/ros2/release-alpha7/ros2.repos -o ros2.repos

# PowerShell
> cd \dev\ros2
> curl https://raw.githubusercontent.com/ros2/ros2/release-alpha7/ros2.repos -o ros2.repos
```

The format of the name of the release comes from the tag associated with the release [here](https://github.com/ros2/ros2/tags).

### Development branches
If you wish to checkout the bleeding-edge development code, you can get the relevant repository list by running:

#### _Linux/OS X_
```
cd ~/ros2_ws
mv -i ros2.repos ros2.repos.old
wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
```

#### _Windows_
```
# CMD
> cd \dev\ros2
> curl -sk https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos -o ros2.repos

# PowerShell
> cd \dev\ros2
> curl https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos -o ros2.repos
```


## Update your repositories
You will notice that in the [`ros2.repos` file](https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos), each repository has a `version` associated with it that points to a particular commit hash, tag, or branch name.
It is possible that these versions refer to new tags/branches that your local copy of the repositories will not recognize as they are out-of-date.
Because of this, you should update the repositories that you have already checked out with the following command:

```
vcs custom --args remote update
```


## Download the new source code
You should now be able to download the sources associated with the new repository list with:

#### _Linux/OS X_
```
vcs import src < ros2.repos
```

#### _Windows_
```
# CMD
> vcs import src < ros2.repos

# PowerShell
> vcs import --input ros2.repos src
```

## Rebuild your workspace
Now that the workspace is up to date with the latest sources, remove your previous install and rebuild your workspace with, for example:

#### _Linux/OS X_
```
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
```

#### _Windows_
```
python src\ament\ament_tools\scripts\ament.py build
```

## Inspecting your source checkout
During your development you may have deviated from the original state of your workspace from when you imported the repository list.
If you wish to know the versions of the set of repositories in your workspace, you can export the information using the following command:

#### _Linux/OS X_
```
cd ~/ros2_ws
vcs export src > my_ros2.repos
```

#### _Windows_
```
> cd \dev\ros2
> vcs export src > my_ros2.repos
```

This `my_ros2.repos` file can then be shared with others so that they can reproduce the state of the repositories in your workspace.
