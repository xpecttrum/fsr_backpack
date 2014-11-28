cmake_minimum_required(VERSION 2.8.3)
project(gy80_md_driver)

find_package(catkin REQUIRED COMPONENTS roscpp tf metal_detector_msgs cereal_port)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${cereal_port_INCLUDE_DIRS})

add_executable(gy80_md_driver_node src/gy80_md_driver.cpp)
target_link_libraries(gy80_md_driver_node ${catkin_LIBRARIES} cereal_port)

#add_executable(set_coils_zero2 src/set_coils_zero.cpp)
#target_link_libraries(set_coils_zero2 ${catkin_LIBRARIES})
