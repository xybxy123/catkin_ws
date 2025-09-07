cmake_minimum_required(VERSION 3.0.2)
project(chassis_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  forward_command_controller
  geometry_msgs
  control_toolbox
  dynamic_reconfigure
  std_msgs
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

generate_dynamic_reconfigure_options(
  config/PID.cfg
)

catkin_package(
  INCLUDE_DIRS 
  include
  LIBRARIES 
  CATKIN_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  forward_command_controller
  geometry_msgs
  control_toolbox
  dynamic_reconfigure
  std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    )
    add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)