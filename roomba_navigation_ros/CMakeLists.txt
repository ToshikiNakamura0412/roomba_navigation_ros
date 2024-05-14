cmake_minimum_required(VERSION 3.8)
project(roomba_navigation_ros)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(obstacle_filter_node src/obstacle_filter.cpp)
target_link_libraries(obstacle_filter_node ${catkin_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
