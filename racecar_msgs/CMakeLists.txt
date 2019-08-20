cmake_minimum_required(VERSION 2.8.3)
project(racecar_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  )

add_message_files(
  DIRECTORY msg
  FILES
  ChassisCommand.msg
  MPCState.msg
  Obstacle.msg
  ObstacleEvent.msg
  Obstacles.msg
  Stage.msg
  Checkline.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetCheckline.srv
  QueryStage.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  )

catkin_package()
