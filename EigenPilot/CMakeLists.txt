project(EigenPilot VERSION 0.0.1 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)

set(TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/basics.cpp)
add_executable(eigen_basics ${TEST_SOURCES})
target_include_directories(eigen_basics PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(eigen_basics Eigen3::Eigen)

add_executable(eigen_quaternion ${CMAKE_CURRENT_SOURCE_DIR}/src/quaternion.cpp)
target_include_directories(eigen_quaternion PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(eigen_quaternion Eigen3::Eigen)
