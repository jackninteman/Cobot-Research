# cobot
Repo for cobot research using C++ and Python. The current setup works for Ubuntu 20.04 LTS and ROS noetic. Using any other configuration may or may not require changes to the codebase.

## Dependencies
Install the following packages before clonning this repo.
```
sudo apt update
sudo apt install ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-kdl-parser libeigen3-dev ros-noetic-joy
```
Running this project also requires successful installations of both libfranka and franka_ros. Carefully follow the instructions provided in `docs/html/installation_linux.html` to complete the installation. 

**Note:** When setting up the real-time kernel for libfranka, the instructions recommend choosing the version closest to the one you currently use. I have experienced extreme difficulty with this method, and I strongly recommend simply choosing the version that was tested with Ubuntu 20.04 (or whichever version of Ubuntu you are using). The tested versions are provided in the tutorial.

## Build Instructions

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/jackninteman/Cobot-Research.git
cd ..
```
If you want to only compile this package, use the following command,
```
catkin_make --only-pkg-with-deps cobot
```
otherwise, compile everything in the catkin workspace,
```
catkin_make
```

## Launch Project (Simulation)
```
roslaunch franka_gazebo franka_gazebo.launch
```
**Note:** When you install `franka_ros` it will include its own `franka_gazebo` package. Since we are using a modified version of this package, you will need to remove the `franka_gazebo` folder from the `franka_ros` directory in order to successfully build your workspace. I would recommend keeping a copy of the original `franka_gazebo` package that you removed from `franka_ros`.

## Launch Project (Experimental)
```
roslaunch cobot_experimental_controller panda_cobot_controller.launch robot_ip:=<robot ip address> load_gripper:=true robot:=panda
```
