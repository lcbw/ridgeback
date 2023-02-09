cmake_minimum_required(VERSION 3.5.0)

project(ridgeback_control)

# Default to C++14

if(NOT CMAKE_CXX_STANDARD)

  set(CMAKE_CXX_STANDARD 14)

endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)

endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

#include_directories(include
#  ${controller_interface_INCLUDE_DIRS}
#  ${urdf_INCLUDE_DIRS}
#  ${realtime_tools_INCLUDE_DIRS}
#  ${tf2_INCLUDE_DIRS}
#  ${nav_msgs_INCLUDE_DIRS}
#  ${rclcpp_INCLUDE_DIRS})​

add_library(mecanum_drive_controller SHARED

  src/mecanum_drive_controller.cpp

  src/odometry.cpp

  src/speed_limiter.cpp)

#target_link_libraries(mecanum_drive_controller​
#  ${controller_interface_LIBRARIES​}
#  ${nav_msgs_LIBRARIES​}​
#  ${rclcpp_LIBRARIES​}​
#  ${realtime_tools​_LIBRARIES​}
#  ${tf2_LIBRARIES​}​
#  ${urdf_LIBRARIES​})

ament_target_dependencies(mecanum_drive_controller​
  controller_interface
  nav_msgs
  rclcpp
  realtime_tools
  tf2
  urdf)

target_include_directories(mecanum_drive_controller​ SYSTEM PUBLIC
    ${controller_interface_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS})​

install(TARGETS mecanum_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES mecanum_drive_controller_plugins.xml
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(controller_interface realtime_tools) ## transitive dependencies - you shouldn't need to findpackage these libararies yourself if you're just using *this, here*

ament_export_include_directories(include) ## what other packages need when linking against this library (headers)

ament_export_libraries(mecanum_drive_controller)  ## what other packages need when linking against this library (source files)

ament_package()
