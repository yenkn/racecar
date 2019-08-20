add_library(sbpl_planner sbpl_planner.cpp)
target_link_libraries(sbpl_planner ${catkin_LIBRARIES} ${SBPL_LIBRARIES})
add_dependencies(sbpl_planner ${catkin_EXPORTED_TARGETS})
target_compile_options(sbpl_planner PUBLIC "-Wno-terminate")  # suppress warning from included SBPL header
