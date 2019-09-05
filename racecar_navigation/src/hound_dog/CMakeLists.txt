add_executable(hound_dog hound_dog.cpp)
target_link_libraries(hound_dog ${catkin_LIBRARIES} ${Boost_LIBRARIES} laser_line)
