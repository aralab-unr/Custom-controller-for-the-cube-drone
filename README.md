This is the ROS simulation and the details of the custom controller of the cube drone for the flying mode, running mode, and climbing mode.
# Software requirement & Setup
This custom controller is designed in:
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
  
Follow these commands to install the package:
```shell
# step 1: clone this repo to your workspace
cd ~/catkin_ws/src
git clone --recursive https://github.com/aralab-unr/Custom-controller-for-the-cube-drone.git

# step 2: follow the standard catkin_make
cd ~/catkin_ws
catkin_make
# step 3: source environment variable
source 
```
# Contents
This section shows how to run the cube-drone and the results of each mode **flying**, **running**, and **climbing**
### a. **Flying mode:**   
An overview of the flying controller frameworks is shown as follows:
<p align='center'>
    <img src="cubedrone/images/schemecubeflying.png" />
</p>

```
# start ros server
roscore
# start gazebo simulator

# start flying controller

```
### b. **Moving mode:**   

```
# start ros server
roscore
# start gazebo simulator

# start moving controller

```
### c. **Climbing mode:**   
An overview of the climbing controller frameworks is shown as follows:
<p align='center'>
    <img src="cubedrone/images/schemecubeclimbing.png" />
</p>

```
# start ros server
roscore
# start gazebo simulator

# start flying controller

```
