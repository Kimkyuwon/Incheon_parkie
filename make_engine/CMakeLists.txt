cmake_minimum_required(VERSION 3.10)  # CMake 버전 요구 사항 설정
project(IncheanAirport)  # 프로젝트 이름 설정

# C++17 표준 사용
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")

# OpenCV 라이브러리와 헤더 파일 경로 설정
find_package(OpenCV REQUIRED)

# CUDA 라이브러리와 헤더 파일 경로 설정
find_package(CUDA REQUIRED)

# Tensor 라이브러리와 헤더 파일 경로 설정 -> 인국공pc 환경설정 자동으로 안찾아져서 set 필요 
# set(TensorRT_Dir "/home/robot/TensorRT-8.5.2.2") #A6000 PC
set(TensorRT_Dir "/usr/local/TensorRT-8.5.2.2") #인국공PC
set(TensorRT_LIB_Dir "${TensorRT_Dir}/targets/x86_64-linux-gnu/lib")  # need to set your own tensorRT directory
set(TensorRT_INCLUDE_Dir "${TensorRT_Dir}/include/")  # need to set your own tensorRT directory

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${TensorRT_INCLUDE_Dir}
)
# Executable 파일 및 소스 파일 설정
add_executable(wheel_detector src/wheel_detector.cpp)
add_executable(onnx_engine src/onnx_engine.cpp)

#링크할 라이브러리 설정
target_link_libraries(wheel_detector
    ${OpenCV_LIBS}                                                                                 
    ${CUDA_LIBRARIES}
    ${TensorRT_LIB_Dir}/libnvparsers.so
    ${TensorRT_LIB_Dir}/libnvinfer.so
    ${TensorRT_LIB_Dir}/libnvonnxparser.so
)

target_link_libraries(onnx_engine
     ${OpenCV_LIBS}
     ${CUDA_LIBRARIES}
     ${TensorRT_LIB_Dir}/libnvparsers.so
     ${TensorRT_LIB_Dir}/libnvinfer.so
     ${TensorRT_LIB_Dir}/libnvonnxparser.so
 )
