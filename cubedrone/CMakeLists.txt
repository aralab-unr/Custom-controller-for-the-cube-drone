cmake_minimum_required(VERSION 2.8.3)
project(cubedrone)
set(CMAKE_CXX_STANDARD 17)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(gazebo REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  dynamic_reconfigure
  gazebo_plugins                    
  gazebo_ros 
  genmsg
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(cubecontroller_node src/cubecontroller.cpp)
add_dependencies(cubecontroller_node ${cubecontroller_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(movingcubedrone src/movingcubedrone.cpp)
add_dependencies(movingcubedrone ${movingcubedrone_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(cuberangefinder src/cuberangefinder.cpp)
add_dependencies(cuberangefinder ${cuberangefinder_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include_directories(${GAZEBO_INCLUDE_DIRS})            
link_directories(${GAZEBO_LIBRARY_DIRS})               
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# for compiling plugin
add_library(cubedrone_plugin SHARED src/cubedrone_plugin.cc)
target_link_libraries(cubedrone_plugin ${GAZEBO_LIBRARIES})
target_link_libraries(cubecontroller_node ${catkin_LIBRARIES})
target_link_libraries(movingcubedrone ${catkin_LIBRARIES})
target_link_libraries(cuberangefinder ${catkin_LIBRARIES})