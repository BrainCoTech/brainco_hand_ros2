cmake_minimum_required(VERSION 3.16)
project(brainco_hand_ethercat_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(stark_ethercat_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)

# Create the plugin library
add_library(${PROJECT_NAME}
  src/revo2_joints_system_slave.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "stark_ethercat_interface"
  "pluginlib"
  "rclcpp"
  "hardware_interface"
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(stark_ethercat_interface revo2_plugins.xml)

# Install include files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the plugin library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch, config, description and script files
install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# # Install plugin description
# install(FILES
#   revo2_plugins.xml
#   DESTINATION share/${PROJECT_NAME}
# )

# Export package information
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
