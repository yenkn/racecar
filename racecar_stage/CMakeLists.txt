cmake_minimum_required(VERSION 2.8.3)
project(racecar_stage)

find_package(catkin REQUIRED COMPONENTS
  rospy)

catkin_package(
  INCLUDE_DIRS scripts
  LIBRARIES
  CATKIN_DEPENDS
  rospy
  DEPENDS
)
