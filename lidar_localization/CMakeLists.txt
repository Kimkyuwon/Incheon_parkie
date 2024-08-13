cmake_minimum_required(VERSION 3.5)
project(lidar_localization)

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
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
# changed
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2 REQUIRED)
# others
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(Ceres REQUIRED)
find_package(Boost COMPONENTS timer)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/pclompm)
set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/pclompm/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
)

add_executable(ndtLocalization src/ndtLocalization.cpp)
target_link_libraries(ndtLocalization ${ament_LIBRARIES} ${PCL_LIBRARIES} # ${CERES_LIBRARIES} 
${OpenMP_CXX_FLAGS} pclompm)
ament_target_dependencies(ndtLocalization
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
  TARGETS ndtLocalization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch rviz config map
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)
ament_package()
