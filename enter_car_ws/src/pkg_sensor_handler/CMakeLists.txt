cmake_minimum_required(VERSION 3.5)
project(pkg_sensor_handler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(ZED 3 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  # "/usr/local/zed/include"  
)

add_executable(node_zed_camera_handler src/node_zed_camera_handler.cpp)

# Link dependencies
target_link_libraries(node_zed_camera_handler
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${ZED_LIBRARIES}
)
# ROS2 specific
ament_target_dependencies(node_zed_camera_handler rclcpp sensor_msgs cv_bridge OpenCV)
  
install(TARGETS node_zed_camera_handler
  DESTINATION lib/${PROJECT_NAME})
  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
