add_executable(graph_planner graph_planner.cpp)
target_link_libraries(graph_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} libtinysplinecpp.a)
