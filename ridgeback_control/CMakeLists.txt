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
    generate_parameter_library
    Eigen3
    geometry_msgs
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

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  mecanum_drive_controller_parameters
  config/mecanum_drive_controller_parameters.yaml
)

add_library(mecanum_drive_controller SHARED
  src/mecanum_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp)

target_link_libraries(mecanum_drive_controller PUBLIC mecanum_drive_controller_parameters)
ament_target_dependencies(mecanum_drive_controller PUBLIC
${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(mecanum_drive_controller SYSTEM PUBLIC
  include
    ${controller_interface_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS})

target_compile_definitions(mecanum_drive_controller PRIVATE "MECANUM_DRIVE_CONTROLLER_BUILDING_DLL")
#pluginlib_export_plugin_description_file(controller_interface mecanum_drive_controller_plugins.xml)

install(FILES mecanum_drive_controller_plugins.xml
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/mecanum_drive_controller
  DESTINATION include/mecanum_drive_controller)

install(TARGETS mecanum_drive_controller mecanum_drive_controller_parameters
#  EXPORT export_mecanum_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  ament_add_gmock(test_mecanum_drive_controller
    test/test_mecanum_drive_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_mecanum_drive_controller.yaml)
  target_include_directories(test_mecanum_drive_controller PRIVATE include)
  target_link_libraries(test_mecanum_drive_controller
    mecanum_drive_controller
  )

  ament_target_dependencies(test_mecanum_drive_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(
    test_load_mecanum_drive_controller
    test/test_load_mecanum_drive_controller.cpp
  )
  target_include_directories(test_load_mecanum_drive_controller PRIVATE include)
  ament_target_dependencies(test_load_mecanum_drive_controller
    controller_manager
  )
  target_compile_definitions(test_load_mecanum_drive_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  ament_add_gmock(
    test_accumulator
    test/test_accumulator.cpp
  )

  target_include_directories(test_accumulator PRIVATE include)
  ament_target_dependencies(test_accumulator)

endif()


#ament_export_targets(export_mecanum_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface realtime_tools hardware_interface) ## transitive dependencies - you shouldn't need to findpackage these libararies yourself if you're just using *this, here*
#ament_export_include_directories(include) ## what other packages need when linking against this library (headers)
ament_export_libraries(mecanum_drive_controller)  ## what other packages need when linking against this library (source files)

ament_package()
