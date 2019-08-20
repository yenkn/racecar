# 神锋无影 Sectumsempra

NCU 14th Racecar, use officially provided hardware to build a car with high performance, which can do mapping and navigation robustly.

## Usage
**Master** -> Mini Computer on the car
Mapping and navigation core runs on the car.

**Client** -> User Computer
Run rviz for surveillance and control.



### Install

for both side

```shell
mkdir ~/catkin_ws
cd ~/catkin_ws
git clone https://github.com/yenkn/racecar.git src
sh src/install.sh
```



### Run Gazebo Simulator

```shell
roslaunch racecar_gazebo racecar_empty_world.launch

# circle task
roslaunch racecar_gazebo racecar_circle_task.launch
```



### Connect to master

```shell
source ./racecar_util/connect.sh # for this session only

# restore setting
source ~/.bashrc # or .zshrc
```




## Structure

TBD