set(PROJECT_VERSION "0.0.1")
project(gflags_example
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

find_package(GFlags REQUIRED)
set(SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${GFLAGS_LIBRARIES})
