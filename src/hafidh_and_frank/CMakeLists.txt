cmake_minimum_required(VERSION 2.8.3)
project(hafidh_and_frank)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    franka_hw
    hardware_interface
    roscpp
    rospy
    realtime_tools
    pluginlib
    dynamic_reconfigure
    message_generation
    geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/compliance_param.cfg
    cfg/desired_mass_param.cfg
)

catkin_package(
    LIBRARIES hafidh_and_frank
    CATKIN_DEPENDS
        controller_interface
        dynamic_reconfigure
        franka_hw
        geometry_msgs
        hardware_interface
        message_runtime
        pluginlib
        realtime_tools
        roscpp
    DEPENDS Franka
)

add_library(hafidh_and_frank
    src/hafidhs_first_controller.cpp
)

add_dependencies(hafidh_and_frank
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
    ${PROJECT_NAME}_gencpp
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(hafidh_and_frank PUBLIC
    ${Franka_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_include_directories(hafidh_and_frank SYSTEM PUBLIC
    ${Franka_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(hafidh_and_frank PUBLIC
    include
)

install(TARGETS hafidh_and_frank
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES hafidh_and_frank_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#catkin_install_python(
#    PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

include (${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
    RESULT_VARIABLE CLANG_TOOLS
)
if (CLANG_TOOLS)
    file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
    file(GLOB_RECURSE HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    )
    add_format_target(hafidh_and_frank FILES ${SOURCES} ${HEADERS})
    add_tidy_target(hafidh_and_frank
        FILES ${SOURCES}
        DEPENDS hafidh_and_frank
    )
endif()