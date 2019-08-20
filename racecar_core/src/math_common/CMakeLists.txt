add_library(math_common math_utils.cpp table_interpolation.cpp)
target_link_libraries(math_common ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(table_interpolation_test test/table_interpolation_test.cpp)
  target_link_libraries(table_interpolation_test ${catkin_LIBRARIES} math_common)
endif()
