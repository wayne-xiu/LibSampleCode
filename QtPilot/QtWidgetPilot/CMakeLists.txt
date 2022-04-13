project(QtWidgetPilot LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(QtWidgetPilot src/Source.cpp)

target_link_libraries(QtWidgetPilot PUBLIC Qt::Widgets)