find_package(PCL REQUIRED)

add_executable(map_localization map_localization_node.cpp)
target_link_libraries(map_localization ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
