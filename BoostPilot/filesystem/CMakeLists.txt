set(PROJECT_VERSION "0.0.1")
project(FilesystemPilot
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

find_package(Boost COMPONENTS system filesystem REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
# target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})
# for some reason, we have to link explicitly; https://stackoverflow.com/questions/1065672/how-to-link-against-boost-system-with-cmake/1065729#1065729
target_link_libraries(${PROJECT_NAME}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY})
