#!/bin/bash

# Varibles
rosversion="kinetic"
# Install the ros

if [ `id -u` == 0 ]; then
	echo "Don't running this use root(sudo)."
	exit 0
fi

echo "Start to install the ros, http://wiki.ros.org/$rosversion/Installation/Ubuntu"
echo "Update the software list"
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update

echo "Install the ros from apt" 
sudo apt-get install ros-$rosversion-desktop-full -y
sudo rosdep init
rosdep update

echo "Setup the ROS environment variables"
echo -e "if [ -f /opt/ros/kinetic/setup.$0 ]; then\n\tsource /opt/ros/kinetic/setup.$0\nfi" >> "~/.$0rc"
echo "source ~/racecar/devel/setup.$0" >> "~/.$0rc"
source "~/.$0rc"

echo "Install the rosinstall"
sudo apt-get install python-rosinstall -y

echo "Install the ssh"
sudo apt-get install ssh -y

echo "Install the ntpdate"
sudo apt-get install ntpdate -y

echo "Install the chrony"
sudo apt-get install chrony -y

echo "Install required models for gazebo"
model_path=~/.gazebo/models
mkdir -p $model_path
wget -nv -O - http://models.gazebosim.org/sun/model.tar.gz | tar -C $model_path -xv
wget -nv -O - http://models.gazebosim.org/ground_plane/model.tar.gz | tar -C $model_path -xv

# Install the dependecies for the project 
echo "Start to config for the project"

#echo "Install the python dependecies"
#sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose -y

#echo "Install the eigen3"
#sudo apt install libeigen3-dev -y

#echo "Install the nlopt"
#sudo apt install libnlopt* -y

echo "Start to config MPC"
sudo apt-get install cppad gfortran -y
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.11.zip && unzip Ipopt-3.12.11.zip && rm Ipopt-3.12.11.zip
sudo ./src/racecar_util/install_ipopt.sh ./Ipopt-3.12.11
rm -rf ./Ipopt-3.12.11

echo "Install the ROS package for racecar"
sudo apt-get install ros-$rosversion-joy -y
sudo apt-get install ros-$rosversion-move-base -y
sudo apt-get install ros-$rosversion-mrpt* -y
sudo apt-get install ros-$rosversion-geographic-msgs -y
sudo apt-get install ros-$rosversion-map-server -y
sudo apt-get install ros-$rosversion-gmapping -y
sudo apt-get install ros-$rosversion-amcl -y
sudo apt-get install ros-$rosversion-rviz-imu-plugin -y
sudo apt-get install ros-$rosversion-dwa-local-planner -y
sudo apt-get install ros-$rosversion-ackermann-msgs -y
sudo apt-get install ros-$rosversion-ros-control ros-$rosversion-ros-controllers ros-$rosversion-gazebo-ros-control -y
sudo apt-get install ros-$rosversion-global-planner ros-$rosversion-sbpl -y
sudo apt-get install ros-$rosversion-robot-localization -y
sudo apt-get install libgsl-dev libgsl2 ros-kinetic-csm -y

echo "--Finish"
