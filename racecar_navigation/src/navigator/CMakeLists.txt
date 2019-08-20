add_executable(navigator_node navigator_node.cpp)
target_link_libraries(navigator_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} path_planner sbpl_planner)
