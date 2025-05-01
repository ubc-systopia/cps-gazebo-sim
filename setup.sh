#!/bin/bash
set -e

cd "$(dirname "$0")"

# Install ROS 2 Humble
if ! dpkg-query -W -f='${Status}' ros-humble-desktop 2>/dev/null | grep -q "install ok installed"; then
    # https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
    echo "Installing ROS2 humble..."
    
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe
    sudo apt update
    sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    echo "Updating apt repository caches..."
    sudo apt update
    sudo apt upgrade -y

    echo "Desktop install: ROS, RViz, demos, tutorials."
    sudo apt install ros-humble-desktop -y
    echo "Development tools: Compilers and other tools to build ROS packages."
    sudo apt install ros-dev-tools -y

    echo "ROS 2 Humble installed successfully."
else
    echo "ROS 2 Humble already installed. Skipping."
fi

# Initialize rosdep (if not already done)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
    rosdep update
else
    echo "rosdep already initialized. Skipping."
fi


# Install Gazebo Sim (via ros-gz)
if ! dpkg-query -W -f='${Status}' ros-${ROS_DISTRO}-ros-gz 2>/dev/null | grep -q "install ok installed"; then
    echo "Installing Gazebo Fortress via ros-gz bridge..."
    export ROS_DISTRO=humble
    sudo apt update
    sudo apt install -y ros-${ROS_DISTRO}-ros-gz
  
    echo "Gazebo Fortress installed successfully."
else
    echo "Gazebo Fortress (ros-gz) already installed. Skipping."
fi

# -----------------------------------------------------------
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

export WS=ros2_ws
export VENDOR_PATH=$WS/src/vendors
mkdir -p $VENDOR_PATH

# Create all vendor subdirectories
mkdir -p "$VENDOR_PATH/universal_robots"
mkdir -p "$VENDOR_PATH/interbotix"
mkdir -p "$VENDOR_PATH/ned2"

echo "Importing vendor repos (see .repos for details)..."
vcs import $VENDOR_PATH/universal_robots < .repos/universal_robots.repos
vcs import $VENDOR_PATH/interbotix < .repos/interbotix.repos
vcs import $VENDOR_PATH/ned2 < .repos/ned2.repos

echo "Installing ROS dependencies..."
cd $WS
rosdep install --from-paths src --ignore-src -r -y

echo "Building colcon workspace..."
colcon build --symlink-install

# Source custom workspace
if [ -f install/setup.bash ]; then
    echo "Sourcing custom workspace..."
    source install/setup.bash
fi

echo "Setup completed."
