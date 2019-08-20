find_package(Eigen3 REQUIRED)

link_directories(/usr/local/lib)

add_executable(mpc_controller mpc.cpp mpc_node.cpp)
add_dependencies(mpc_controller racecar_msgs_gencpp)
target_link_libraries(mpc_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} ipopt math_common)
