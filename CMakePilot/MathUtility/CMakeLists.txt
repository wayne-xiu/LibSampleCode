add_library(MathUtility
    src/addition.cpp
    src/division.cpp)

# target_include_directories(MathUtility PUBLIC include)
target_include_directories(MathUtility PUBLIC
$<INSTALL_INTERFACE:include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/MathUtility/addition.h
            ${CMAKE_CURRENT_SOURCE_DIR}/include/MathUtility/division.h
            DESTINATION ${CMAKE_INSTALL_PREFIX}/include/MathUtility)

install(TARGETS MathUtility EXPORT my_export DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/MathUtility)
install(EXPORT my_export FILE MathUtility-config.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/MathUtility)