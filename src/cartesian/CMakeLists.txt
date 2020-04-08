cmake_minimum_required(VERSION 2.8.3)
project(cartesian)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    std_msgs
    roscpp
)

catkin_package(
    LIBRARIES cartesian
    CATKIN_DEPENDS
        geometry_msgs
        std_msgs
        roscpp
)

add_executable(cartesian src/processor.cpp)

target_link_libraries(cartesian
    ${catkin_LIBRARIES}
)

target_include_directories(cartesian SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)

add_dependencies(cartesian
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

install(TARGETS cartesian
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

