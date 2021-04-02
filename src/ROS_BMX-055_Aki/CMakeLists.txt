cmake_minimum_required(VERSION 3.0.2)
project(ros_bmx_055_aki)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES test
    #  CATKIN_DEPENDS roscpp rospy std_msgs tf
    #  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(imu_data_to_pose src/imu_data_to_pose.cpp ) 
target_link_libraries(imu_data_to_pose ${catkin_LIBRARIES} )

add_executable(mag_data_to_pose src/mag_data_to_pose.cpp ) 
target_link_libraries(mag_data_to_pose ${catkin_LIBRARIES} )

