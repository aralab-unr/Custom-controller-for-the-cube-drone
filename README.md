# Cube-Drone ROS Simulation and Custom Controller

This repository contains the ROS simulation and details of the custom controller for the cube-drone. It clarifies the contents of the manuscript: [Cube_drone__ICRA_ARA.pdf](https://github.com/user-attachments/files/16533043/Cube_drone__ICRA_ARA.pdf).

## Software Requirements & Setup

The custom controller is designed in:

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

Follow these commands to install the package:

```shell
# Step 1: Create and build a catkin workspace:
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/
$ catkin_make
$ echo "source ~/dev_ws/devel/setup.bash" >> ~/.bashrc

# Step 2: Clone this repo into your workspace
$ cd ~/dev_ws/src
$ git clone --recursive https://github.com/aralab-unr/Custom-controller-for-the-cube-drone.git

# Step 3: Build the catkin workspace for this package
$ cd ~/dev_ws
$ catkin_make
```
# Contents
This section shows how to run the simulation and the results of each **flying**, **running**, and **climbing** mode. The cube-drone parameters are imported from URDF file as:

| Parameter | Value | Parameter | Value |
| --- | --- | --- | --- 
| m | 2.8 $(kg)$ | $k_t$ | $2.5.10^{-4}$ $(N.s^2)$ |
| $I_{xx}$ | 0.0044 $(kg.m^2)$ | $k_d$ | $7.5.10^{-5}$ $(Nm.s^2)$ |
| $I_{yy}$ | 0.0023 $(kg.m^2)$ | $l_x$ | $0.1845 (m)$ |
| $I_{zz}$ | 0.0058 $(kg.m^2)$ | $l_y$ |  $0.219 (m)$ |
| Sampling time | 0.05 $(s)$ | 


More details of each mode and its algorithm can be found in this file:
[Clarification of the custom controller for the Cube-Drone.pdf](https://github.com/user-attachments/files/16827489/Clarification.of.the.custom.controller.for.the.Cube-Drone.pdf)


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
roslaunch cubedrone gazebo.launch
# start flying controller
rosrun cubedrone cubecontroller_node
```

https://github.com/user-attachments/assets/a751fc7a-9035-4bb7-8fea-c892e7abaf63

The parameters of the flying controller are as follows:

| Parameter | Value | Parameter | Value |
| --- | --- | --- | --- 
| $k_{px}$ | 0.1 | $\text{PID}_z$ | P = 8; I = 0; D = 5.75 |
| $k_{dx}$ | 0.15 | $\text{PID}_{\phi}$ | P = 0.08; I = 0; D = 0.05 |
| $k_{py}$ | 0.1 | $\text{PID}_{\theta}$ | P = 0.08; I = 0; D = 0.05|
| $k_{dy}$ | 0.15 | $\text{PID}_{\psi}$ | P = 0.05; I = 0; D = 0.04 |
| $x_r $ | 5 m | $y_r $ | 5 m |
| $z_r $ | 5 m | $\psi_r $ | 0 rad |

### b. **Moving mode:**   
Follow these commands in order to run the moving mode:

```
# start ros server
roscore
# start gazebo simulator
roslaunch cubedrone gazebo.launch
# start moving controller
rosrun cubedrone movingcubedrone
```
The moving controller is designed based on this algorithm:
<p align='left'>
    <img src="cubedrone/images/algorithmcube.png" width="450" />
</p>

https://github.com/user-attachments/assets/f47c8db2-a66a-4f89-b168-1131a725187e

The parameters of the moving controller are as follows:
| Parameter | Value | Parameter | Value |
| --- | --- | --- | --- 
| $\text{PID}_{\psi}$ | P = 2.5; I = 0; D = 1.4 | $\text{PID}_{v}$ | P = 50.4; I = 0.05; D = 15.4 |
| $x_r$ | 7.5 m | $y_r$ | 5 m |

### c. **Climbing mode:**   
An overview of the climbing controller framework is shown as follows:
<p align='center'>
    <img src="cubedrone/images/schemecubeclimbing.png" />
</p>

```
Follow these commands in order to run the climbing mode:

# start ros server
roscore
# start gazebo simulator
roslaunch cubedrone rangefinder.launch
# start flying controller
rosrun cubedrone cuberangefinder
```
The parameters of the climbing controller are as follows:
| Parameter | Value | Parameter | Value |
| --- | --- | --- | --- 
| $\phi_r$ | 0 rad  | $\theta_r $ | 0 rad |
| $\psi_r$ | 0 rad | $z_r$ | 1.5 m |
| $\text{PID}_z$ | P = 8; I = 0; D = 5.75 | $\text{PID}_{\psi}$ | P = 0.05; I = 0; D = 0.04 |
| $\text{PID}_{\phi}$ | P = 0.5; I = 0; D = 0.4  | $\text{PID}_{v}$ | P = 0.75 ; I = 0; D = 0|
| $\text{PID}_{\theta}$ | P = 0.5; I = 0; D = 0.4 |



https://github.com/user-attachments/assets/c17747fc-f272-4769-bdf9-9989708ab5bc

