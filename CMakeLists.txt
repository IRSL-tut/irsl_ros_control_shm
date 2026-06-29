cmake_minimum_required(VERSION 3.15)
project(irsl_ros_control_shm
  VERSION 0.99.1
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(irsl_shm_libs)
find_package(dynamixel_hardware_shm QUIET)

add_library(${PROJECT_NAME} SHARED
  src/RobotHWShm.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME}
  irsl_shm_libs::irsl_shm_libs
)

if(dynamixel_hardware_shm_FOUND)
  message(STATUS "BUILD: RobotDxHWShm plugin with dynamixel_hardware_shm")
  target_sources(${PROJECT_NAME} PRIVATE src/RobotDxHWShm.cpp)
  target_link_libraries(${PROJECT_NAME}
    dynamixel_hardware_shm::dynamixel_hardware_shm
  )
endif()

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  src/RobotHWShm.h
  src/RobotDxHWShm.h
  src/joint_info.h
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
