cmake_minimum_required(VERSION 3.5.0)

project(ridgeback_control)

# Default to C++14

if(NOT CMAKE_CXX_STANDARD)

  set(CMAKE_CXX_STANDARD 14)

endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)

endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
#    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    builtin_interfaces
    urdf
    tf2
    tf2_msgs
    nav_msgs
    rcpputils
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

#find_package(controller_interface REQUIRED)
#find_package(generate_parameter_library REQUIRED)
#find_package(builtin_interfaces REQUIRED)
#find_package(hardware_interface REQUIRED)
#find_package(urdf REQUIRED)
#find_package(realtime_tools REQUIRED)
#find_package(tf2 REQUIRED)
#find_package(nav_msgs REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(rcpputils REQUIRED)
#find_package(pluginlib REQUIRED)
#find_package(rclcpp_lifecycle REQUIRED)
#find_package(std_msgs REQUIRED)

#include_directories(include
#  ${controller_interface_INCLUDE_DIRS}
#  ${urdf_INCLUDE_DIRS}
#  ${realtime_tools_INCLUDE_DIRS}
#  ${tf2_INCLUDE_DIRS}
#  ${nav_msgs_INCLUDE_DIRS}
#  ${rclcpp_INCLUDE_DIRS})​
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#generate_parameter_library(
#  mecanum_drive_controller_parameters
#  config/mecanum_drive_controller_parameters.yaml
#)

add_library(mecanum_drive_controller SHARED
  src/mecanum_drive_controller.cpp
  src/mecanum_drive_controller_base.cpp
  src/odometry.cpp
  src/speed_limiter.cpp)

#target_link_libraries(mecanum_drive_controller​
#  ${controller_interface_LIBRARIES​}
#  ${nav_msgs_LIBRARIES​}​
#  ${rclcpp_LIBRARIES​}​
#  ${realtime_tools​_LIBRARIES​}
#  ${tf2_LIBRARIES​}​
#  ${urdf_LIBRARIES​})

ament_target_dependencies(mecanum_drive_controller
${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(mecanum_drive_controller SYSTEM PUBLIC
  include
    ${controller_interface_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS})

target_link_libraries(mecanum_drive_controller
    mecanum_drive_controller_parameters
    )

install(TARGETS mecanum_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES mecanum_drive_controller_plugins.xml
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(controller_interface realtime_tools hardware_interface) ## transitive dependencies - you shouldn't need to findpackage these libararies yourself if you're just using *this, here*

ament_export_include_directories(include) ## what other packages need when linking against this library (headers)

ament_export_libraries(mecanum_drive_controller)  ## what other packages need when linking against this library (source files)

ament_package()
