cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

# Project
set(PROJECT_NAME 3PSDF)
set(CMAKE_CXX_STANDARD 11)
get_filename_component(PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(EXTERNAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/lib)
set(EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen)
set(PQP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/PQP)
set(MANIFOLD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/manifold)

# libigl
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(LIBIGL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/libigl)

find_package(LIBIGL REQUIRED QUIET)
find_package(OpenMP REQUIRED)

# includes
include_directories(${EXTERNAL_DIR})
include_directories(${INCLUDE_DIR})
include_directories(${INCLUDE_DIR}/apps)
include_directories(${LIBIGL_DIR}/include)
# include_directories(${LIBIGL_DIR}/external/glad/include)
include_directories(${LIBIGL_DIR}/external)
# include_directories(${LIBIGL_DIR}/external/libigl-imgui)
# include_directories(${LIBIGL_DIR}/external/imgui/examples)
include_directories(${EIGEN_DIR})
include_directories(${PQP_DIR})
include_directories(${MANIFOLD_DIR})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  message(STATUS "Build type not specified: Use Release by default.")
endif(NOT CMAKE_BUILD_TYPE)

# Source Files
file(GLOB SRC_FILES ${PROJECT_DIR}/src/*.cpp)
file(GLOB HEADER_FILES ${PROJECT_DIR}/src/*.h)
file(GLOB PQP_FILES ${PQP_DIR}/*.cpp)
file(GLOB MANIFOLD_FILES ${MANIFOLD_DIR}/*.cpp)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

# Add executable to build
add_executable(batchGenOctL3PSDF 
  ${PROJECT_DIR}/src/batchGenOctL3PSDF.cpp
)
target_link_libraries(batchGenOctL3PSDF stdc++fs)

add_executable(genOctreeL3PSDFSamples
  ${PROJECT_DIR}/src/genOctreeL3PSDFSamples.cpp ${PROJECT_DIR}/src/lib/computeLocal3PoleSDF.cpp 
  ${PROJECT_DIR}/src/lib/utilities.cpp 
  ${PROJECT_DIR}/src/lib/distComputeUtility.cpp  ${PQP_FILES} ${MANIFOLD_FILES}
)
target_link_libraries(genOctreeL3PSDFSamples PRIVATE "${OpenMP_CXX_FLAGS}" stdc++fs igl::core)
target_compile_options(genOctreeL3PSDFSamples PRIVATE "${OpenMP_CXX_FLAGS}")
