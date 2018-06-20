cmake_minimum_required(VERSION 2.8.3)
project(pcl_sample)

add_compile_options(-std=c++11)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# OpenMP
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  message("OpenMP Found.")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Eigen
include_directories(${EIGEN3_INCLUDE_DIRS})

# PCL
find_package(PCL 1.8 REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# CUDA
find_package(CUDA REQUIRED QUIET)
include_directories(${CUDA_INCLUDE_DIRS})
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")

# Boost
find_package(Boost 1.40 COMPONENTS program_options REQUIRED QUIET)
include_directories(${Boost_INCLUDE_DIR})

include_directories(include)

add_executable(../bin/marching_cubes ../src/marching_cubes.cpp)
target_link_libraries(../bin/marching_cubes
  ${PCL_LIBRARIES} ${Boost_LIBRARIES})

cuda_add_executable(../bin/marching_cubes_gpu ../src/marching_cubes_gpu.cpp src/cuda/marching_cubes.cu)
target_link_libraries(../bin/marching_cubes_gpu
  ${PCL_LIBRARIES})

add_executable(../bin/depth_to_cloud ../src/depth_to_cloud.cpp)
target_link_libraries(../bin/depth_to_cloud
  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(../bin/view_obj_or_pcd ../src/view_obj_or_pcd.cpp)
target_link_libraries(../bin/view_obj_or_pcd
  ${PCL_LIBRARIES} ${Boost_LIBRARIES})
