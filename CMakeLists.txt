cmake_minimum_required(VERSION 3.8)
project(kassow_kord_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ENABLE_TEST FALSE)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(THIS_PACKAGE_DEPS
  assimp
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
  find_package(${PACKAGE} REQUIRED)
endforeach()

add_library(
  kassow_kord_driver
  SHARED
  src/kassow_kord_driver.cpp
)
target_include_directories(
  kassow_kord_driver
  PUBLIC
  include
  # ${KASSOW_VENDOR_INCLUDE_DIR}
)
ament_target_dependencies(
  kassow_kord_driver
  ${THIS_PACKAGE_DEPS}
)

# Link your driver library
# target_link_libraries(kassow_kord_driver
#   ${KASSOW_VENDOR_LIBRARY}
# )

# prevent pluginlib from using boost
target_compile_definitions(kassow_kord_driver PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface kassow_kord_driver.xml)

install(
  TARGETS
  kassow_kord_driver
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(ENABLE_TEST)
  ament_add_gmock(test_kassow_kord_driver test/test_kassow_kord_driver.cpp)
  target_include_directories(test_kassow_kord_driver PRIVATE include)
  ament_target_dependencies(
    test_kassow_kord_driver
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  kassow_kord_driver
)
ament_export_dependencies(
  hardware_interface
  ${THIS_PACKAGE_DEPS}
)

ament_package()
