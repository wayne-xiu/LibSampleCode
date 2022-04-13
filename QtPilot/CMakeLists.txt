cmake_minimum_required(VERSION 3.10)

set(PROJECT_VERSION "0.0.1")
project(QtPilot
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GUN" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(warnings "/W4 /WX /EHsc")
endif()

if(${UNIX})
     set(CMAKE_PREFIX_PATH 
     "/opt/Qt/6.2.4/gcc_64/lib/cmake"
)
else()
	set(CMAKE_PREFIX_PATH
		"C:/Qt/6.2.4/msvc2017_64/lib/cmake"
)
endif()

find_package(QT NAMES Qt6 COMPONENTS Core Gui Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Gui Widgets REQUIRED)

add_subdirectory(QtConsolePilot)
add_subdirectory(QtWidgetPilot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)
# Set build type if not set
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${default_build_type})
endif()