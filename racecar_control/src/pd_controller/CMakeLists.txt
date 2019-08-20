# find_package (Eigen3 3.3.90 REQUIRED NO_MODULE)
add_executable(pd_controller pd_controller.cpp)
target_link_libraries(pd_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pd_controller ${PROJECT_NAME}_gencfg)
