add_library(serial_port serial_port.cpp serial_port_threaded.cpp serial_common.cpp)
target_link_libraries(serial_port ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(serial_port racecar_msgs_gencpp)

install(TARGETS
  serial_port
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
