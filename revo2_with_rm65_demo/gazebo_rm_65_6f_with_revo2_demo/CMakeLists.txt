cmake_minimum_required(VERSION 3.8)
project(gazebo_rm_65_6f_with_revo2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  urdf
  meshes
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
