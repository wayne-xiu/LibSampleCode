cmake_minimum_required(VERSION 3.13)

project(CeresPilot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")

# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

add_executable(ceresHelloWorld src/helloWorld.cpp)
target_link_libraries(ceresHelloWorld ${CERES_LIBRARIES})

add_executable(powell src/powell.cpp)
target_link_libraries(powell ${CERES_LIBRARIES})

add_executable(curveFitting src/curveFitting.cpp)
target_link_libraries(curveFitting ${CERES_LIBRARIES})

add_executable(robustCurveFitting src/robustCurveFitting.cpp)
target_link_libraries(robustCurveFitting ${CERES_LIBRARIES})

add_executable(simpleBundleAdjuster src/simpleBundleAdjuster.cpp)
target_link_libraries(simpleBundleAdjuster ${CERES_LIBRARIES})
