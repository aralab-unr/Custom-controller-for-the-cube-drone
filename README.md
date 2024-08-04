This is the ROS simulation and the details of the custom controller of the cube drone for the flying mode, running mode, and climbing mode.
# Software requirement
This custom controller is designed in:
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Solidworks 2023
Installing the package with the following commands:
```shell
# step1: install dependencies
sudo apt install ros-${ROS_DISTRO}-octomap* && sudo apt install ros-${ROS_DISTRO}-mavros* && sudo apt install ros-${ROS_DISTRO}-vision-msgs

# step 2: clone this repo to your workspace
cd ~/catkin_ws/src
git clone --recursive https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy.git

# optional: switch to simulation branch for autonomous_flight
# the default branch is for real flight and PX4 simulation
cd path/to/autonomous_flight
git checkout simulation

# step 3: follow the standard catkin_make procedure
cd ~/catkin_ws
catkin_make
```
