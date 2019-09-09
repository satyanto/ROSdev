cmake_minimum_required(VERSION 2.8.3)
project(hafidh_controller)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    franka_hw
    geometry_msgs
    hardware_interface
    message_generation
    pluginlib
    realtime_tools
    roscpp
    rospy
)

find_package(Franka 0.5.0 REQUIRED)

generate_messages()

catkin_package(
    LIBRARIES hafidh_controller
    CATKIN_DEPENDS
        controller_interface
        franka_hw
        geometry_msgs
        hardware_interface
        message_runtime
        pluginlib
        realtime_tools
        roscpp
    DEPENDS Franka
)

add_library(hafidh_controller
    src/hafidh_controller.cpp
)

add_dependencies(hafidh_controller
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
    ${PROJECT_NAME}_gencpp
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(hafidh_controller PUBLIC
    ${Franka_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_include_directories(hafidh_controller SYSTEM PUBLIC
    ${Franka_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(hafidh_controller PUBLIC
    include
)



install(TARGETS hafidh_controller
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

install(FILES hafidh_controller_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
    RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
    file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
    file(GLOB_RECURSE HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    )
    add_format_target(hafidh_controller FILES ${SOURCES} ${HEADERS})
    add_tidy_target(hafidh_controller
        FILES ${SOURCES}
        DEPENDS hafidh_controller
    )
endif()
