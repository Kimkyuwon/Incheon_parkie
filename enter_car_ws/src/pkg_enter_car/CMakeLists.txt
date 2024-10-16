cmake_minimum_required(VERSION 3.5)
project(pkg_enter_car)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Tensor 라이브러리와 헤더 파일 경로 설정 -> 인국공pc 환경설정 자동으로 안찾아져서 set 필요 
set(TensorRT_Dir "/home/robot/TensorRT-8.5.2.2")
set(TensorRT_LIB_Dir "${TensorRT_Dir}/targets/x86_64-linux-gnu/lib")  # need to set your own tensorRT directory
set(TensorRT_INCLUDE_Dir "${TensorRT_Dir}/include/")  # need to set your own tensorRT directory

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_Dir}
)

add_executable(node_wheel_detection src/node_wheel_detection.cpp)
ament_target_dependencies(node_wheel_detection rclcpp sensor_msgs cv_bridge OpenCV)
# target_include_directories(node_wheel_detection PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

target_link_libraries(node_wheel_detection
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${TensorRT_LIB_Dir}/libnvparsers.so
  ${TensorRT_LIB_Dir}/libnvinfer.so
  ${TensorRT_LIB_Dir}/libnvonnxparser.so
)

install(TARGETS node_wheel_detection
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
