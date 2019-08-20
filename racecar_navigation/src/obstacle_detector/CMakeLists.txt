find_package(OpenCV REQUIRED)

add_executable(obstacle_detector obstacle_detector.cpp)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} laser_line)

add_executable(matching_detector matching_detector.cpp)
target_link_libraries(matching_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES})
