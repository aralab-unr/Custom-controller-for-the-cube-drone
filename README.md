This is the ROS simulation and the details of the custom controller of the cube-drone. This repository is to clarify this manuscript: [Cube_drone__ICRA_ARA.pdf](https://github.com/user-attachments/files/16533043/Cube_drone__ICRA_ARA.pdf).

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
This section shows how to run the simulation and the results of each **flying**, **running**, and **climbing** mode.
### a. **Flying mode:**   
An overview of the flying controller framework is shown as follows:
<p align='center'>
    <img src="cubedrone/images/schemecubeflying.png" />
</p>
Follow these commands in order to run the flying mode:

```
# start ros server
roscore
# start gazebo simulator

# start flying controller

```



https://github.com/user-attachments/assets/a751fc7a-9035-4bb7-8fea-c892e7abaf63

The parameter of the flying controller is as follows:

| Command | Description |
| --- | --- |
| git status | List all new or modified files |
| git diff | Show file differences that haven't been staged |

### b. **Moving mode:**   

```
# start ros server
roscore
# start gazebo simulator

# start moving controller

```



https://github.com/user-attachments/assets/f47c8db2-a66a-4f89-b168-1131a725187e


### c. **Climbing mode:**   
An overview of the climbing controller framework is shown as follows:
<p align='center'>
    <img src="cubedrone/images/schemecubeclimbing.png" />
</p>

```
# start ros server
roscore
# start gazebo simulator

# start flying controller

```


https://github.com/user-attachments/assets/c17747fc-f272-4769-bdf9-9989708ab5bc

