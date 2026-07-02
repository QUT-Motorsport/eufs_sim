#!/bin/bash

cd ~

## Pre-reqs
echo ""
echo "---Installing pre-reqs---"
echo ""
sleep 3
sudo apt update
sudo apt upgrade -y
sudo apt install -y git python3-pip cmake

# for installing pip rosdeps
export PIP_BREAK_SYSTEM_PACKAGES=1

## Make workspace
mkdir ~/QUTMS
cd ~/QUTMS
echo "export QUTMS_WS=~/QUTMS" >> ~/.bashrc
export QUTMS_WS=~/QUTMS

# directory for people to drop their rosbags in
mkdir bags/

## Clone Driverless repo
echo ""
echo "---Cloning Driverless repo---"
echo ""
sleep 3
git clone --recurse-submodules https://github.com/QUT-Motorsport/QUTMS_Driverless.git
cd QUTMS_Driverless

## Set ROS2 repositories
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_ros_source.sh

## Install ROS2
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_ros_jazzy.sh

## Package requirements
echo ""
echo "---Installing driverless package requirements---"
echo ""
sleep 3
pip install -r ~/QUTMS/QUTMS_Driverless/installation/requirements.txt

## Create an alias for ease
echo "alias a='source install/setup.bash'" >> ~/.bashrc

ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/build.sh ~/QUTMS/build.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/format.sh ~/QUTMS/format.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/launch.sh ~/QUTMS/launch.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/pull.sh ~/QUTMS/pull.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/record.sh ~/QUTMS/record.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/play_bag.sh ~/QUTMS/play_bag.sh

## Source ROS
source /opt/ros/jazzy/setup.bash

## Install dependencies from src/
rosdep install --from-paths ~/QUTMS --ignore-src -r -y

## Pre commit for git
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_pre-commit.sh

echo ""
echo "---Building packages---"
echo ""
sleep 3
cd ~/QUTMS

# build the rest of the workspace
source ~/QUTMS/build.sh --all

## Wrap up QUTMS Driverless
echo "Driverless repo installed."

## Eufs sim setup. Installation from the migration branch.
git clone https://github.com/QUT-Motorsport/eufs_sim -b feature/jazzy_migration

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/install/eufs_tracks/share/eufs_tracks/
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/eufs_sim/eufs_tracks/meshes
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/eufs_sim/eufs_tracks/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/eufs_sim/eufs_tracks/worlds
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/eufs_sim/eufs_models/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/install/eufs_tracks/share/eufs_tracks/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/install/eufs_models/share/eufs_models/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/eufs_sim/eufs_tracks/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/eufs_sim/eufs_tracks/worlds
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/eufs_sim/eufs_models/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/eufs_sim/eufs_models/meshes
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/install/eufs_tracks/share/eufs_tracks/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/install/eufs_tracks/share/eufs_tracks/worlds
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/install/eufs_models/share/eufs_models/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$QUTMS_WS/install/eufs_models/share/eufs_models/meshes
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_SIM_RESOURCE_PATH=$QUTMS_WS/eufs_sim/eufs_tracks:$QUTMS_WS/eufs_sim/eufs_tracks/worlds:$QUTMS_WS/eufs_sim/eufs_tracks/models:$QUTMS_WS/eufs_sim/eufs_tracks/meshes:$QUTMS_WS/eufs_sim/eufs_msg/msg/proto/gz/driverless_msg:$QUTMS_WS/build/eufs_msg/msg 
export GAZEBO_RESOURCE_PATH=$QUTMS_WS/eufs_sim/eufs_tracks:$QUTMS_WS/eufs_sim/eufs_tracks/worlds:$QUTMS_WS/eufs_sim/eufs_tracks/models:$QUTMS_WS/eufs_sim/eufs_tracks/meshes:$QUTMS_WS/eufs_sim/eufs_msg/msg/proto/gz/driverless_msg:$QUTMS_WS/build/eufs_msg/msg 
export GAZEBO_MODEL_PATH=$QUTMS_WS 
export EUFS_MASTER=$QUTMS_WS

source /opt/ros/jazzy/setup.bash 
.install/setup.bash

sudo apt install -y python3-pip pre-commit cmake software-properties-common curl mesa-utils
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
sudo apt install ros-jazzy-ros-gz
## launch 
colcon build --packages-up-to eufs_launcher 
.install/setup.bash 
source /opt/ros/jazzy/setup.bash 
ros2 launch eufs_launcher eufs_launcher.launch.py
