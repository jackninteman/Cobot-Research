# cobot
Repo for cobot research using C++ and Python. The current setup only works for Ubuntu 16.04 LTS and ROS Kinetic. 
We are planning to extend this work for Ubuntu 18.04 and ROS Melodic.

## Dependencies
Install the following packages before clonning this repo.
```
sudo apt update
sudo apt install ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-kdl-parser libeigen3-dev ros-melodic-joy
```
## Build Instructions

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/danoponline/cobot.git
cd ..
```
If you want to only compile this package,
```
catkin_make --only-pkg-with-deps cobot
```
otherwise, compile everything in the catkin workspace,
```
catkin_make
```
## Launch Project
```
roslaunch franka_gazebo franka_gazebo.launch
```