cmake_minimum_required(VERSION 2.6)
project(assetswriter_ros)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             nav_msgs
             message_filters
             tf)
             
find_package(Eigen3 REQUIRED)


             
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})


             
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
             
add_executable(src/assetswriter_node main.cpp)


target_link_libraries(assetswriter_node ${catkin_LIBRARIES})

install(TARGETS assetswriter_node RUNTIME DESTINATION bin)

add_subdirectory(data)
