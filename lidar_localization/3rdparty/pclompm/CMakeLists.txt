cmake_minimum_required(VERSION 3.6)
project(pclompm)

set(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(pclompm SHARED src/pclompm/voxel_grid_covariance_omp.cpp src/pclompm/ndt_omp.cpp src/pclompm/gicp_omp.cpp)

target_include_directories(pclompm PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(pclompm ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(pclompm OpenMP::OpenMP_CXX)
else()
    message(WARNING "OpenMP not found")
endif()
