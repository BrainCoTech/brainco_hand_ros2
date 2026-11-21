cmake_minimum_required(VERSION 3.5)
project(revo2_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # 如需添加单元测试，请将测试脚本放置在 test/ 并在此注册
  # ament_add_pytest_test(${PROJECT_NAME}_urdf_tests test/urdf_tests.py)
endif()

# 安装资源目录
install(DIRECTORY
  urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


