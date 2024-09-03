cmake_minimum_required(VERSION 3.5)
project(sensor_handler2)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
# not used
# find_package(message_generation REQUIRED)

find_package(std_msgs   REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# changed
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# others
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost COMPONENTS timer)
find_package(novatel_oem7_msgs REQUIRED)
# find_package(ublox_serialization REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(sensorChecker src/sensorChecker.cpp)
target_link_libraries(sensorChecker ${ament_LIBRARIES})
ament_target_dependencies(sensorChecker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  novatel_oem7_msgs
  PCL
  EIGEN3
)

add_executable(sensorErrorHandler src/sensorErrorHandler.cpp)
target_link_libraries(sensorErrorHandler ${ament_LIBRARIES})
ament_target_dependencies(sensorErrorHandler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  PCL
  EIGEN3
)

add_executable(DR_Handler src/DR_Handler.cpp)
target_link_libraries(DR_Handler ${ament_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(DR_Handler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rclcpp
  rclpy
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  PCL
  EIGEN3
  pcl_conversions
)

add_executable(gnssHandler src/gnssHandler.cpp)
target_link_libraries(gnssHandler ${ament_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(gnssHandler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rclcpp
  rclpy
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  PCL
  EIGEN3
  pcl_conversions
  novatel_oem7_msgs
)

add_executable(lidarHandler src/lidarHandler.cpp)
target_link_libraries(lidarHandler ${ament_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(lidarHandler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rclcpp
  rclpy
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  PCL
  EIGEN3
  pcl_conversions
)

install(
  TARGETS  DR_Handler gnssHandler lidarHandler sensorErrorHandler sensorChecker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)
ament_package()
