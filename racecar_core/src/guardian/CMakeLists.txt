add_executable(guardian guardian.cpp)
add_dependencies(guardian racecar_msgs_gencpp)
target_link_libraries(guardian ${catkin_LIBRARIES} ${Boost_LIBRARIES})
