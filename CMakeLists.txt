cmake_minimum_required(VERSION 3.15)
project(irsl_ros_control_shm
  VERSION 0.99.1
  LANGUAGES CXX
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
#  pluginlib
  angles
  controller_manager
  hardware_interface
  joint_limits_interface
  transmission_interface
  urdf
)

### catkin xxxx
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ros_control_shm
  CATKIN_DEPENDS controller_manager
  #DEPENDS
)

find_package(irsl_shm_libs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(node_shm src/node.cpp src/RobotHWShm.cpp src/print_urdf.cpp)
target_link_libraries(node_shm
  ${catkin_LIBRARIES}
  irsl_shm_libs::irsl_shm_libs
)
# install(TARGETS node_shm
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

install(TARGETS node_shm
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(dynamixel_hardware_shm)
#get_cmake_property(_variableNames VARIABLES)
#message("VARIABLES>>>>")
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#  message("${_variableName}=${${_variableName}}")
#endforeach()
#message("VARIABLES<<<<")
if (${dynamixel_hardware_shm_FOUND})
  message("BUILD: node_dx_shm with dynamixel_hardware_shm")
  add_executable(node_dx_shm src/node_dx.cpp src/RobotDxHWShm.cpp src/RobotHWShm.cpp src/print_urdf.cpp)
  target_link_libraries(node_dx_shm
    ${catkin_LIBRARIES}
    irsl_shm_libs::irsl_shm_libs
    dynamixel_hardware_shm::dynamixel_hardware_shm
  )
  install(TARGETS node_dx_shm
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
#add_library(RobotHWShm src/RobotHWShm.cpp)
#target_link_libraries(RobotHWShm
#  ${catkin_LIBRARIES}
#)
#install(TARGETS RobotHWShm
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
#install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  USE_SOURCE_PERMISSIONS
#  )
