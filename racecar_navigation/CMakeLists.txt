cmake_minimum_required(VERSION 2.8.3)
project(racecar_navigation)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  racecar_msgs
  racecar_core
  costmap_2d
  interactive_markers
  tf
  )

# Find SBPL
find_package(SBPL REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_line
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs racecar_msgs racecar_core costmap_2d
  DEPENDS SBPL
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(include)

add_subdirectory(src/lsm_odom)
add_subdirectory(src/obstacle_detector)
add_subdirectory(src/path_planner)
add_subdirectory(src/sbpl_planner)
# add_subdirectory(src/navigator)
add_subdirectory(src/laser_line)
add_subdirectory(src/map_localization)
add_subdirectory(src/graph_localization)
add_subdirectory(src/graph_planner)
add_subdirectory(src/graph_detector)
add_subdirectory(src/scan_planner)
add_subdirectory(src/hound_dog)

roslaunch_add_file_check(launch)

install(
        DIRECTORY config launch maps
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

