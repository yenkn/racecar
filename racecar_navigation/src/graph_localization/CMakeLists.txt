find_package(OpenCV REQUIRED)

add_executable(graph_localization graph_localization.cpp)
target_link_libraries(graph_localization ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} laser_line)
