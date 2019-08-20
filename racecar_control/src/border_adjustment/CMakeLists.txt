add_executable(border_adjustment border_adjustment.cpp border_adjustment_ros.cpp gradient_path.cpp grid_path.cpp)
add_dependencies(border_adjustment racecar_msgs_gencpp)
target_link_libraries(border_adjustment ${catkin_LIBRARIES} ${Boost_LIBRARIES} math_common)
