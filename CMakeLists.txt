cmake_minimum_required(VERSION 3.8)
project(hardware_interface_wrappers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/transmission_wrapper.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_wrappers_ros2_control_plugins.xml)


# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(
    test_transmission_wrapper
    test/transmission_wrapper_test.cpp
  )
  target_include_directories(test_transmission_wrapper PUBLIC include hardware_interface)
  ament_target_dependencies(test_transmission_wrapper hardware_interface)


  ament_add_gmock(
    test_transmission_wrapper_loader
    test/transmission_wrapper_loader_test.cpp
  )
  target_include_directories(test_transmission_wrapper_loader PUBLIC include hardware_interface)
  ament_target_dependencies(test_transmission_wrapper_loader hardware_interface)

endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  hardware_interface
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
