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

add_executable(midterm_16_easy src/midterm_16_easy.cpp)
target_link_libraries(midterm_16_easy ${catkin_LIBRARIES})
target_link_libraries (midterm_16_easy ${PCL_LIBRARIES})

add_executable(midterm_16_medium src/midterm_16_medium.cpp)
target_link_libraries(midterm_16_medium ${catkin_LIBRARIES})
target_link_libraries (midterm_16_medium ${PCL_LIBRARIES})
