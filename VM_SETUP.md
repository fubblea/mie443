# Virtual Machine Setup

## Installing the VM

1. Uninstall any other VM software
2. Install VMWare Workstation Pro
    - Instructions: https://www.mikeroysoft.com/post/download-fusion-ws/
3. Download the Ubuntu 16 ISO: https://releases.ubuntu.com/16.04/
4. Setup the VM using the default setting for everthing buy make sure to change these:
    - Enable 3D acceleration. Allow of the max video memory available on your GPU
    - At least 8GB of RAM
    - At least 50GB of storage
    - Bump up the number of CPU cores to at least half your total CPU cores.
5. Once the VM has booted up and installed the OS. Reboot before continuing.

**DO NOT INSTALL ANY OS UPDATES!!!**

## Installing Software

If there is some permission error somewhere, just rerun the command with `sudo`. I might have missed a couple here.

### Install Locales

1. `sudo apt-get update && sudo apt-get install -y locales`
2. `locale-gen en_US.UTF-8`
3. `sudo update-locale LANG=en_US.UTF-8`

### Install ROS

1. `sudo apt-get update && sudo apt-get install -y lsb-release && sudo apt-get clean all`
2. `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
3. `sudo apt install curl -y`
4. `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
5. `sudo apt-get update -y`
6. `sudo apt-get install ros-kinetic-desktop-full -y`

If for some reason the above setups didn't work. I would just reinstall the VM. It will save you a lot of pain debugging in the future.

If there have been no errors so far, go ahead and reboot.

### Install Additional Dependencies

1. `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y`
2. `source /opt/ros/kinetic/setup.bash`
3. `rosdep init`
4. `rosdep update`

### Install Turtlebot Packages

1. `sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs -y`
2. `source /opt/ros/kinetic/setup.bash`
3. `rosrun kobuki_ftdi create_udev_rules`
4. `sudo apt-get install ros-kinetic-audio-common -y`

### Update Gazebo to latest 7.x version

1. Add key:
```
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```
2. `sudo apt-get install wget`
3. `wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -`
4. `sudo apt-get update -y`
5. `sudo apt-get install gazebo7 -y`

### Add stuff to bashrc

1. `echo 'source /opt/ros/kinetic/setup.bash' >> /home/$USERNAME/.bashrc`
2. `echo 'export TURTLEBOT_3D_SENSOR=kinect' >> /home/$USERNAME/.bashrc`
3. `echo 'export SVGA_VGPU10=0' >> /home/$USERNAME/.bashrc`

Reboot.

### Check to make sure everything works

1. `sudo apt-get install glmark2`
2. Run `glmark2`, it should say `GL_VERSION` is 2.x. If it's 3.x, try rebooting and check again.
3. Run `gazebo`, you should be getting a decent FPS.

If you get this far and it works, you owe me a beer. It took me wayyy too long to figure this out.

### Install VSCode

1. `sudo apt install aptitude`
2. `sudo apt-get update -y`
3. `sudo aptitude install code`

If 3 does not work try:

4. `sudo snap install --classic code`

Reject other options and select yes when prompted to install the older version of code (1.11.2)

### Install terminator

1. `sudo add-apt-repository -y ppa:gnome-terminator`
2. `sudo apt-get update -y`
3. `sudo apt-get install terminator -y`
