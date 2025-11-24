cmake_minimum_required(VERSION 3.5)
project(robot_api)

if(NOT DEFINED ENV{ROS_VERSION} OR "$ENV{ROS_VERSION}" STREQUAL "1")
  # ROS 1 (catkin)
  add_compile_options(-std=c++14)
  find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    eigen_conversions
    geometric_shapes
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    roscpp
    roslaunch
    sensor_msgs
    robot_api_msgs
  )

  catkin_python_setup()

  catkin_package(
    CATKIN_DEPENDS
      actionlib_msgs
      geometry_msgs
      moveit_msgs
      moveit_ros_planning_interface
      sensor_msgs
      robot_api_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(${PROJECT_NAME}_moveit_macros include/${PROJECT_NAME}/fake_object_recognition.h src/moveit_macros.cpp)
  add_dependencies(${PROJECT_NAME}_moveit_macros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_moveit_macros ${catkin_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_moveit_macros PROPERTIES OUTPUT_NAME moveit_macros PREFIX "")

  install(TARGETS ${PROJECT_NAME}_moveit_macros
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
  )

  install(DIRECTORY
    config
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  roslaunch_add_file_check(launch namespace:=mobipick)

else()
  # ROS 2 (ament)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(geometric_shapes REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(moveit_ros_planning_interface REQUIRED)
  find_package(robot_api_msgs REQUIRED)

  add_executable(${PROJECT_NAME}_moveit_macros_ros2 src/moveit_macros_ros2.cpp)
  ament_target_dependencies(${PROJECT_NAME}_moveit_macros_ros2 rclcpp rclcpp_action std_msgs robot_api_msgs moveit_ros_planning_interface geometric_shapes control_msgs)
  set_target_properties(${PROJECT_NAME}_moveit_macros_ros2 PROPERTIES OUTPUT_NAME moveit_macros_ros2 PREFIX "")

  install(
    TARGETS ${PROJECT_NAME}_moveit_macros_ros2
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )

  ament_python_install_package(${PROJECT_NAME})

  ament_package()
endif()
