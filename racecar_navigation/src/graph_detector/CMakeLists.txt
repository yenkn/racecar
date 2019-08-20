add_executable(graph_detector graph_detector.cpp)
target_link_libraries(graph_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES} laser_line)
