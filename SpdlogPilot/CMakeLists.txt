set(PROJECT_VERSION "0.0.1")
project(SpdlogPilot
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

if (NOT TARGET spdlog)
    find_package(spdlog REQUIRED)
endif()

set(SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)
