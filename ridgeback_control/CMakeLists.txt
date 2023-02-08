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

set(mecanum_drive_controller_sources
  src/mecanum_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mecanum_drive_controller
#  CATKIN_DEPENDS controller_interface realtime_tools
#)

#include_directories(
#  include ${catkin_INCLUDE_DIRS}
#)

add_library(mecanum_drive_controller ${mecanum_drive_controller_headers} ${mecanum_drive_controller_sources})
#target_link_libraries(mecanum_drive_controller ${catkin_LIBRARIES})
ament_target_dependencies(mecanum_drive_controller)

install(TARGETS mecanum_drive_controller
  ARCHIVE DESTINATION lib/${PROJECT_NAME}/
  LIBRARY DESTINATION lib/${PROJECT_NAME}/
  RUNTIME DESTINATION bin/${PROJECT_NAME}/
  )

install(FILES mecanum_drive_controller_plugins.xml
    DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

#if(CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(launch/control.launch)
#  roslaunch_add_file_check(launch/teleop.launch)
#endif()

ament_package()
