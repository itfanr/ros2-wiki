This page describes how to set up a linux machine for ROS2 CI jobs using AWS.

# Creating an AWS instance
In short, use the company AWS account to launch an instance running based off the official Ubuntu 16.04 AMI.

* **AMI:** Ubuntu 16.04
* **Region:** N. California us-west-1a
* **Type:** c4.large
* **Storage:** EBS 1TB
* **Security Group:** `ROS 2 Jenkins Build Machines`
* **Key pair:** Create a new pair with a descriptive name like `ci_ros2_linux_4`
  * Make sure to save it with the other credentials so others can access this machine

Give the instance a descriptive name like `ROS2 CI (linux 4)`.
Record the ip address [here](https://docs.google.com/spreadsheets/d/1OSwqbE3qPF8v3HSMr8JOaJ6r4QOiQFk6pwgaudXVE-4/edit#gid=0) (private).

# Setting up the machine
In short, make sure the jenkins master can ssh into the new node and run docker.

1. Use the key pair to log into the new node
   * `ssh -i ci_ros2_linux_4.pem ubuntu@IPADDRESS`
1. Run the following commands
    
    ```
    sudo apt-get update
    sudo apt-get install -y git
    sudo apt-get install -y openjdk-8-jre-headless
    sudo bash -c 'echo "deb http://repositories.ros.org/ubuntu/testing/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo bash -c 'curl --silent http://repositories.ros.org/repos.key |sudo apt-key add -'
    sudo apt-get update
    sudo apt-get install -y python-vcstool
    curl -fsSL https://get.docker.com/ | sh
    sudo adduser --disabled-password jenkins
    sudo usermod -aG docker jenkins
    sudo service docker start
    ```
1. Make sure the jenkins user can run docker
    
    ```
    sudo su jenkins
    docker run hello-world
    ```
1. As the jenkins user, add the master's public key to `authorized_keys`
    1. SSH into the jenkins master and get the contents of the public key (id_rsa.pub)
    2. SSH into the new slave and add that key to the authorized keylist
    ```
    # on new slave
    cd /home/jenkins/
    mkdir .ssh
    touch .ssh/authorized_keys
    # Paste id_rsa.pub from the jenkins master into this file
    vim .ssh/authorized_keys
    ```

# Adding it to the master
1. Add a new agent to http://ci.ros2.org/computer/
    * **Number of executors:** 1
    * **Remote root directory:** /home/jenkins
    * **Labels:** `linux`
    * **Launch method:** Launch slave agents via ssh
      * **Host:** Ip address of new node
      * **Credentials:** `Jenkins`
      * **Host Key Verification Strategy:** `Manually provided key verification strategy`
        * **SSH Key** paste the contents of `/etc/ssh/ssh_host_rsa_key.pub` from the new node here.
     * **Node Properties:**
       * Check `Notify when Node online status changes` and set the email to the ros2 buildfarm google group.
2. Launch the agent on the new node