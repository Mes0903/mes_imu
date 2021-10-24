cmake_minimum_required(VERSION 2.8.3)
project(mes_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
    LIBRARIES mes_imu
    CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mes_imu src/mes_imu.cpp)

target_link_libraries(mes_imu
  ${catkin_LIBRARIES}
  RTIMULib
)

catkin_package()
