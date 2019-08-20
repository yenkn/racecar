# find_package (Eigen3 3.3.90 REQUIRED NO_MODULE)
add_executable(pure_pursuit pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pure_pursuit ${PROJECT_NAME}_gencfg)
