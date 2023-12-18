set(PROJECT_VERSION "0.0.1")
project(InstallationCheck
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

# if(${UNIX})
#     set(Boost_INCLUDE_DIR /usr/local/src/boost_1_70_0)
#     set(Boost_LIBRARY_DIR /usr/local/src/boost_1_70_0/stage/lib)
# else()
#     set(Boost_INCLUDE_DIR C:/boost_1.70.0)
#     set(Boost_LIBRARY_DIR C:/boost_1.70.0/stage/v140/x64/lib)
# endif()

# find_package(Boost COMPONENTS chrono system filesystem REQUIRED)
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

include_directories("${CMAKE_SOURCE_DIR}/include")

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})
