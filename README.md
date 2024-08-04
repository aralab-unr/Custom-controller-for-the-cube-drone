This is the ROS simulation and the details of the custom controller of the cube drone for the flying mode, running mode, and climbing mode.
# Software requirement & Setup
This custom controller is designed in:
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Solidworks 2023
  
Following these commands to install the package:
```shell
# step 1: clone this repo to your workspace
cd ~/catkin_ws/src
git clone --recursive https://github.com/aralab-unr/Custom-controller-for-the-cube-drone.git

# step 2: follow the standard catkin_make procedure
cd ~/catkin_ws
catkin_make
# step 3: source environment variable
source 
```
# Contents
This section shows how to run the cube-drone and the results of each mode **flying**, **running**, and **climbing**
### a. **Flying mode:**   
```
# start simulator
roslaunch uav_simulator start.launch # recommand to use the corridor env for your first trial

# open the Rviz visualization
roslaunch remote_control dynamic_navigation_rviz.launch # if your test env has dynamic obstacles

# run the navigation program
roslaunch autonomous_flight dynamic_navigation.launch # if your test env has dynamic obstacles

# --------------------------------------------------------------------------------------
# (alternatively, if your test env is purely static, you can run the following instead)
# open the Rviz visualization
roslaunch remote_control navigation_rviz.launch # if your test env only has static obstacles

# run the navigation program
roslaunch autonomous_flight navigation.launch # if your test env only has static obstacles
```
