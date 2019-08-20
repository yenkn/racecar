add_executable(racecar_chassis racecar_chassis.cpp)
add_dependencies(racecar_chassis racecar_msgs_gencpp)
target_link_libraries(racecar_chassis ${catkin_LIBRARIES} ${Boost_LIBRARIES} serial_port)
