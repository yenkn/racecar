add_executable(scan_planner scan_planner.cpp)
target_link_libraries(scan_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} path_planner sbpl_planner)
