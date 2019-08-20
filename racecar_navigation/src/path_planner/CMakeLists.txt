add_library(path_planner
  quadratic_calculator.cpp
  dijkstra.cpp
  astar.cpp
  grid_path.cpp
  gradient_path.cpp
  orientation_filter.cpp
  planner_core.cpp
)
add_dependencies(path_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner ${catkin_LIBRARIES})

add_executable(path_planner_node
  plan_node.cpp
)
add_dependencies(path_planner_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner_node
  path_planner
  ${catkin_LIBRARIES}
)
