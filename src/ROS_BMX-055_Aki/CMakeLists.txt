cmake_minimum_required(VERSION 3.0.2)
project(ros_bmx-055_aki)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_data_to_pose src/imu_data_to_pose.cpp ) 
target_link_libraries(imu_data_to_pose ${catkin_LIBRARIES} )

