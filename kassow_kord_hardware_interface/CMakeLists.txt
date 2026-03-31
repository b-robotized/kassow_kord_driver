cmake_minimum_required(VERSION 3.8)
project(kassow_kord_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ENABLE_TEST FALSE)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(THIS_PACKAGE_DEPS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  kassow_kord_vendor
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
  find_package(${PACKAGE} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/kassow_kord_hardware_interface.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    kord-api::kord-api
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface kassow_kord_hardware_interface.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(ENABLE_TEST)
  ament_add_gmock(test_kassow_kord_hardware_interface test/test_kassow_kord_hardware_interface.cpp)
  target_include_directories(test_kassow_kord_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_kassow_kord_hardware_interface
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  ${THIS_PACKAGE_DEPS}
)

ament_package()
