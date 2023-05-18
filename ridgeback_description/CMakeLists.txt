cmake_minimum_required(VERSION 3.5.0)
project(ridgeback_description)

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
#find_package(catkin REQUIRED COMPONENTS roslaunch)

#catkin_package()

install(DIRECTORY env-hooks meshes launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/env_run
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ridgeback_description.dsv.in")

#if(CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(launch/description.launch)
#endif()
ament_package()
