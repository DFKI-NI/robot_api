cmake_minimum_required(VERSION 3.16)
project(robot_api_msgs)

if(NOT DEFINED ENV{ROS_VERSION} OR "$ENV{ROS_VERSION}" STREQUAL "1")

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    geometry_msgs
    message_generation
)

add_action_files(
    DIRECTORY action
    FILES
    MoveItMacro.action
    FtObserver.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime
)

else()
# compile with C++14
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Register your action files
rosidl_generate_interfaces(${PROJECT_NAME}
  action/FtObserver.action
  action/MoveItMacro.action
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
endif()
