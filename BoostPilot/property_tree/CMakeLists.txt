set(PROJECT_VERSION "0.0.1")
project(PropertyTreePilot
    VERSION ${PROJECT_VERSION}
    LANGUAGES CXX)

find_package(Boost 1.70 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
# target_link_libraries(${PROJECT_NAME} PRIVATE Boost::property_tree)
