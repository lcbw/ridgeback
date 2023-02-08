cmake_minimum_required(VERSION 3.5.0)
project(ridgeback_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch maps params
  DESTINATION share/${PROJECT_NAME}/
)

#if(CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(launch/include/amcl.launch)
#  roslaunch_add_file_check(launch/amcl_demo.launch)
#  roslaunch_add_file_check(launch/include/gmapping.launch)
#  roslaunch_add_file_check(launch/gmapping_demo.launch)
#  roslaunch_add_file_check(launch/include/move_base.launch)
#  roslaunch_add_file_check(launch/odom_navigation_demo.launch)
#endif()
