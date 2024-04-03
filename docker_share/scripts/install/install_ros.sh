#!/bin/bash

# Installation for ROS 2
# Ex: source install_ros.sh pha humble

IN_USERNAME=$1
IN_ROS_VERSION=$2
IN_USERNAME="${IN_USERNAME:=pha}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
source /home/${IN_USERNAME}/.bashrc

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y

if [ "${IN_ROS_VERSION}" == "foxy" ]; then
    sudo apt install ros-$IN_ROS_VERSION-desktop -y 
else
    sudo apt install ros-$IN_ROS_VERSION-desktop -y
fi

sudo apt install ros-dev-tools -y

sudo apt install python3-colcon-common-extensions -y

echo -e "\n# ROS 2" >> /home/${IN_USERNAME}/.bashrc
echo "source /opt/ros/$IN_ROS_VERSION/setup.bash" >> /home/${IN_USERNAME}/.bashrc
source /opt/ros/$IN_ROS_VERSION/setup.bash
mkdir -p /home/${IN_USERNAME}/ros2_ws/src
cd /home/${IN_USERNAME}/ros2_ws
python3 -m pip install virtualenv
python3 -m virtualenv -p python3 ./venv
touch ./venv/COLCON_IGNORE

sudo rosdep init
rosdep update

cd src
mkdir tutorials
cd tutorials
git clone https://github.com/ros2/examples -b $IN_ROS_VERSION
cd /home/${IN_USERNAME}/ros2_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $IN_ROS_VERSION # $ROS_DISTRO
colcon build --symlink-install
echo "source /home/${IN_USERNAME}/ros2_ws/install/setup.bash" >> /home/${IN_USERNAME}/.bashrc
echo -e "\n# Colon" >> /home/${IN_USERNAME}/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/${IN_USERNAME}/.bashrc
echo "export _colcon_cd_root=/opt/ros/$IN_ROS_VERSION/" >> /home/${IN_USERNAME}/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${IN_USERNAME}/.bashrc
echo -e "\n# Venv" >> /home/${IN_USERNAME}/.bashrc
echo -e "\n# source /home/${IN_USERNAME}/ros2_ws/venv/bin/activate" >> /home/${IN_USERNAME}/.bashrc
cd /home/${IN_USERNAME}
source /home/${IN_USERNAME}/.bashrc