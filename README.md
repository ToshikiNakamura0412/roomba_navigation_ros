# roomba_navigation_ros

![Build Status](https://github.com/ToshikiNakamura0412/roomba_navigation_ros/workflows/build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A 2D navigation package for Roomba

## Environment
- Ubuntu 20.04
- ROS Noetic

## Requirement
- python3-vcstool

## Dependencies
- [amr_navigation_ros](https://github.com/ToshikiNakamura0412/amr_navigation_ros.git)
- [Roomba](https://github.com/amslabtech/Roomba.git)

## Install and Build
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/ToshikiNakamura0412/roomba_navigation_ros.git
cd roomba_navigation_ros
vcs import . < .rosinstall
cd amr_navigation_ros
vcs import navigation < .rosinstall

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic # Install dependencies
catkin build -DCMAKE_BUILD_TYPE=Release                 # Release build is recommended
```
