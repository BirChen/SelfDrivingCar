cmake_minimum_required(VERSION 2.8.3)
project(midterm_16)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  tf
)
find_package(PCL 1.2 REQUIRED)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(midterm_16 src/midterm_16.cpp)
target_link_libraries(midterm_16 ${catkin_LIBRARIES})
target_link_libraries (midterm_16 ${PCL_LIBRARIES})
