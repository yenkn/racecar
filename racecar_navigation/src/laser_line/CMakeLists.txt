add_library(laser_line laser_feature_ros.cpp line_feature.cpp)

target_link_libraries(laser_line ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS laser_line
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)