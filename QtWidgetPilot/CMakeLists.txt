cmake_minimum_required(VERSION 3.15...3.18)
project(QtWidgetPilot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(${UNIX})
     set(CMAKE_PREFIX_PATH 
		"/home/Qt5.14.0/5.14.0/gcc_64/lib/cmake/Qt5"
)
else()
	set(CMAKE_PREFIX_PATH
		"C:/Qt/5.14.0/msvc2017_64/lib/cmake/Qt5"
)
endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)
# add_executable(helloworld
#    mainwindow.ui
#    mainwindow.cpp
#    main.cpp
#    resources.qrc
#)

add_executable(QtWidgetPilot src/Source.cpp)

target_link_libraries(QtWidgetPilot Qt5::Widgets)