set(PROJECT_VERSION "0.0.1")
project(BoostTestPilot
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

find_package(Boost 1.70.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})


set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})
