#!/bin/bash

# Installation for ROS 2
# First input [$1] is the Username - ex: pha
# Second input [$2] is the ROS Vesion - ex: humble

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y

sudo apt install ros-$2-desktop-full -y

sudo apt install ros-dev-tools -y

sudo apt install python3-colcon-common-extensions -y

echo -e "\n # ROS 2" >> /home/$1/.bashrc
echo "source /opt/ros/$2/setup.bash" >> /home/$1/.bashrc
source /opt/ros/$2/setup.bash
mkdir -p /home/$1/ros2_ws/src
cd /home/$1/ros2_ws
python3 -m virtualenv -p python3 ./venv
touch ./venv/COLCON_IGNORE

sudo rosdep init
rosdep update

cd src
mkdir tutorials
cd tutorials
git clone https://github.com/ros2/examples -b $2
cd /home/$1/ros2_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $2 # $ROS_DISTRO
colcon build --symlink-install
echo "source /home/$1/ros2_ws/install/setup.bash" >> /home/$1/.bashrc
echo -e "\n # Colon" >> /home/$1/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$1/.bashrc
echo "export _colcon_cd_root=/opt/ros/$2/" >> /home/$1/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$1/.bashrc
cd /home/$1
souce /home/$1/.bashrc