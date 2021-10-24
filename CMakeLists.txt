cmake_minimum_required(VERSION 3.5)
project(mes_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  rospy
)

catkin_package(
    LIBRARIES mes_imu
    CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mes_imu_pub src/mes_imu_pub.cpp)
target_link_libraries(
  mes_imu_pub
  ${catkin_LIBRARIES}
  RTIMULib
)

add_executable(mes_imu_lis src/mes_imu_lis.cpp)
target_link_libraries(
  mes_imu_lis
  ${catkin_LIBRARIES}
  RTIMULib
)

catkin_package()