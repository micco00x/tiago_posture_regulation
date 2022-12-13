# TIAGo Posture Regulation
## Installation
To make sure everything is working properly make sure you have Ubuntu 18.04 with
ROS Melodic. Install catkin_tools, create a catkin workspace and clone this
repository in the `src` folder. Make sure you are compiling in *Release* mode
by properly setting your catkin workspace:
```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Build your code by running the following command:
```bash
catkin build
```

## Usage
To run the Gazebo simulation:
```bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=empty
```

To run the posture regulation module:
```bash
roslaunch tiago_posture_regulation tiago_posture_regulation.launch
```
