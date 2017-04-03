# Setup CI server

## Installing

Install the latest LTS release from http://pkg.jenkins-ci.org/debian-stable/

## Running on port 80

I used this SO answer to setup a subdomain to a port:

http://serverfault.com/a/140161/186748

I had to remove the `hudson` in each of the lines that contained it.

## Temporary rewrite for changed job name

We renamed some of the jobs, so I added rewrite rules in Apache (`/etc/apache2/sites-enabled/ci.ros2.org.conf`):

```
# Temporary rewrite rule because we changed the Windows job name.
RewriteEngine On
RewriteRule ^(.*)/ros2_batch_ci_linux/(.*)$ $1/ci_linux/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_osx/(.*)$ $1/ci_osx/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_windows/(.*)$ $1/ci_windows_opensplice/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_windows_opensplice/(.*)$ $1/ci_windows_opensplice/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_windows_connext_static/(.*)$ $1/ci_windows_connext_static/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_windows_connext_dynamic/(.*)$ $1/ci_windows_connext_dynamic/$2 [R=301,L]
RewriteRule ^(.*)/ci_windows_opensplice/(.*)$ $1/ci_windows/$2 [R=301,L]
RewriteRule ^(.*)/ci_windows_connext_static/(.*)$ $1/old_windows_connext_static/$2 [R=301,L]
RewriteRule ^(.*)/ci_windows_connext_dynamic/(.*)$ $1/old_windows_connext_dynamic/$2 [R=301,L]
RewriteRule ^(.*)/ci_windows_fastrtps/(.*)$ $1/old_windows_fastrtps/$2 [R=301,L]

RewriteRule ^(.*)/ros2_batch_ci_linux_nightly/(.*)$ $1/nightly_linux/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_osx_nightly/(.*)$ $1/nightly_osx/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_windows_opensplice_nightly/(.*)$ $1/nightly_windows_opensplice/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_windows_connext_static_nightly/(.*)$ $1/nightly_windows_connext_static/$2 [R=301,L]
RewriteRule ^(.*)/ros2_batch_ci_windows_connext_dynamic_nightly/(.*)$ $1/nightly_windows_connext_dynamic/$2 [R=301,L]
RewriteRule ^(.*)/nightly_windows_opensplice/(.*)$ $1/nightly_windows/$2 [R=301,L]
RewriteRule ^(.*)/nightly_windows_connext_static/(.*)$ $1/old_night_windows_connext_static/$2 [R=301,L]
RewriteRule ^(.*)/nightly_windows_connext_dynamic/(.*)$ $1/old_night_windows_connext_dynamic/$2 [R=301,L]

RewriteRule ^(.*)/ros2_packaging_linux/(.*)$ $1/packaging_linux/$2 [R=301,L]
RewriteRule ^(.*)/ros2_packaging_osx/(.*)$ $1/packaging_osx/$2 [R=301,L]
RewriteRule ^(.*)/ros2_packaging_windows_opensplice/(.*)$ $1/packaging_windows_opensplice/$2 [R=301,L]
RewriteRule ^(.*)/packaging_windows_opensplice/(.*)$ $1/packaging_windows/$2 [R=301,L]
```

## Install stuff (needed on master and slaves)

```
sudo apt-get update
sudo apt-get install -y git
# qemu and vcs are required for ARM builds
sudo apt-get install -y qemu-user-static
sudo bash -c 'echo "deb http://repositories.ros.org/ubuntu/testing/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
sudo bash -c 'curl --silent http://repositories.ros.org/repos.key |sudo apt-key add -'
sudo apt-get update
sudo apt-get install -y python-vcstool
curl -fsSL https://get.docker.com/ | sh
sudo adduser jenkins
sudo usermod -aG docker jenkins
```

## Adding a Linux slave to the farm
Approximately:

* Shell into the master (`ci.ros2.org`), copy `/var/lib/jenkins/.ssh/id_rsa.pub` and paste it into `/home/jenkins/.ssh/authorized_keys` on the new machine.
* Copy config from the `linux 2` machine, rename and otherwise modify as needed (e.g., change the IP/host).

## Configuring Jenkins

First I updated all the preinstalled  plugins.

### Authentication

Then I setup authentication with the `github-oauth` plugin.
I just installed it and followed their setup instructions:

https://wiki.jenkins-ci.org/display/JENKINS/Github+OAuth+Plugin

I created an application entry on the ros2 GitHub organization:

https://github.com/organizations/ros2/settings/applications/215300

I also tuned the permissions in `Manage Jenkins->Configure Global Security`.

### Plugins

Next I installed all of these plugins:

- `ansicolor`
- `description-setter`
- `github` (other git* plugins are deps of the `github-oauth` plugin)
- `greenballs`
- `groovy`
- `parameterized-trigger`
- `PrioritySorter`
- `jobrequeue`
- `ssh-agent`
- `warnings`
- `xunit`

### Adding an ssh key

Jenkins needs a valid ssh key in order to pull from some of our private repositories, for example to get the rti deb files.

So I created an ssh key for the jenkins user on the webserver:

```
sudo su jenkins
cd
mkdir .ssh
ssh-keygen -t rsa
```

Then I added to the jenkins credentials as an "From the jenkins master ~/.ssh" with the user id of `ros2-buildfarm`.

I added this key to a "machine" GitHub account that I created for this farm and I added that user, `ros2-buildfarm`, to the `ros2`, `ament`, and `osrf` organizations.

## Creating Jobs

I cloned the `ros2/ci` repository to the default branch (`master`):

```
git clone https://github.com/ros2/ci.git
```

Then I cloned the `ros_buildfarm` repository:

```
git clone https://github.com/ros-infrastructure/ros_buildfarm.git
```

I also install the `jenkinsapi` and `EmPy` Python packages:

```
sudo apt-get install python3-pip
sudo -H python3 -m pip install -U pip
sudo -H python3 -m pip install jenkinsapi EmPy
```

Then I setup auth:

```
mkdir -p ~/.buildfarm
vim ~/.buildfarm/jenkins.ini
```

Put this in the `jenkins.ini` file:

```
[http://ci.ros2.org]
username=wjwwood
password=<your application token>
```

Now, you should first login with GitHub on Jenkins if you haven't already.
Then put your github username in and for the application token, browse to the configuration of your user on Jenkins:

http://ci.ros2.org/user/wjwwood/configure

In those settings there should be a field called API Token.
Copy that field for your password.

Now I can create the jobs:

```
$ PYTHONPATH=`pwd`/../ros_buildfarm ./create_jenkins_job.py -u http://ci.ros2.org
Connecting to Jenkins 'http://ci.ros2.org'
Connected to Jenkins version '1.617'
Creating job 'ros2_batch_ci_windows'
The Jenkins master does not require a crumb
Creating job 'ros2_batch_ci_osx'
Creating job 'ros2_batch_ci_linux'
Creating job 'ros2_batch_ci_launcher'
```

### Tuning Auto-generated Jobs

The final step is to fine tune the jobs.
For each job you'll want to check the ssh key being used for the git clone (only on Linux) and the ssh-agent.
It should be set to the ssh key setup in the previous steps for the jenkins user.

I also updated the slaves which the jobs will run on to make sure they matched the names of the slaves I added for Linux, OS X and Windows.

## Disk space

Over time docker images and particularly containers will pile up.
To clean up use:

```
docker rm $(docker ps -a -q)
docker rmi $(docker images -q -f dangling=true)
```

from https://www.calazan.com/docker-cleanup-commands/
