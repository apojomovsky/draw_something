cmake_minimum_required(VERSION 2.8.3)
project(draw_something)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS geometry_msgs std_msgs std_srvs)

add_executable(draw_something src/draw_something.cpp)

target_link_libraries(draw_something ${catkin_LIBRARIES} ${Boost_LIBRARIES})

