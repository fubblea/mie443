FROM ubuntu:16.04

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# Use bash
SHELL ["/bin/bash", "-c"]

# Install locales
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8

# Install ROS
RUN apt-get update && apt-get install -y lsb-release && apt-get clean all \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt install curl -y \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update -y \
    && apt-get install ros-kinetic-desktop-full -y

# Install additional deps
RUN apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y \
    && source /opt/ros/kinetic/setup.bash \
    && rosdep init \
    && rosdep update

# Install turtlebot packages
RUN apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs -y \
    && source /opt/ros/kinetic/setup.bash \
    # && rosrun kobuki_ftdi create_udev_rules \
    && apt-get install ros-kinetic-audio-common -y

# Update Gazebo version to latest 7.x
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && apt-get install wget \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update -y \
    && apt-get install gazebo7 -y

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/kinetic/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'export TURTLEBOT_3D_SENSOR=kinect' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/devel/setup.bash' >> /home/$USERNAME/.bashrc

# Setup entrypoint
ENTRYPOINT ["/bin/bash"]
