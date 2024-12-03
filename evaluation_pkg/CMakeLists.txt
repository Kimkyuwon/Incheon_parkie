cmake_minimum_required(VERSION 3.8)
project(evaluation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(std_msgs   REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# changed
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# others
find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)
find_package(Boost COMPONENTS timer)


add_executable(evaluation_node src/evaluation_node.cpp)
target_link_libraries(evaluation_node ${ament_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(evaluation_node
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
  EIGEN3
)

install(TARGETS evaluation_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
