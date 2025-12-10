# =============================================================================
# BrainCo Hand Driver - CMakeLists.txt
# =============================================================================
# BrainCo 机械手驱动程序构建配置

cmake_minimum_required(VERSION 3.16)
project(brainco_hand_driver)

# -----------------------------------------------------------------------------
# 编译器警告设置
# -----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# BrainCo Stark SDK 配置
# SDK 位于 vendor 目录，由 download_sdk.sh 脚本预先下载
# -----------------------------------------------------------------------------
set(BRAINCO_STARK_SDK_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
set(BRAINCO_STARK_SDK_INCLUDE_DIR "${BRAINCO_STARK_SDK_VENDOR_DIR}/dist/include")
set(BRAINCO_STARK_SDK_LIB_DIR "${BRAINCO_STARK_SDK_VENDOR_DIR}/dist/shared/linux")
set(BRAINCO_STARK_SDK_LIB_PATH "${BRAINCO_STARK_SDK_LIB_DIR}/libbc_stark_sdk.so")

# -----------------------------------------------------------------------------
# 验证 SDK 文件完整性
# -----------------------------------------------------------------------------
if(NOT EXISTS ${BRAINCO_STARK_SDK_INCLUDE_DIR}/stark-sdk.h)
  message(FATAL_ERROR "stark-sdk.h not found in ${BRAINCO_STARK_SDK_INCLUDE_DIR}. Please run './download_sdk.sh' first.")
endif()

if(NOT EXISTS ${BRAINCO_STARK_SDK_LIB_PATH})
  message(FATAL_ERROR "BrainCo Stark SDK library not found: ${BRAINCO_STARK_SDK_LIB_PATH}. Please run './download_sdk.sh' first.")
endif()

# -----------------------------------------------------------------------------
# 创建 BrainCo Stark SDK 的 CMake 导入库目标
# SHARED IMPORTED GLOBAL - 共享库、导入的、全局可见
# -----------------------------------------------------------------------------
add_library(brainco_stark_sdk SHARED IMPORTED GLOBAL)
set_target_properties(brainco_stark_sdk PROPERTIES
  IMPORTED_LOCATION ${BRAINCO_STARK_SDK_LIB_PATH}           # 库文件位置
  INTERFACE_INCLUDE_DIRECTORIES ${BRAINCO_STARK_SDK_INCLUDE_DIR}  # 头文件目录
)

# -----------------------------------------------------------------------------
# ZLG USB-CAN FD 驱动库
# 该库随仓库 vendor 目录一同发布，详细请见 https://manual.zlg.cn/web/#/146 
# -----------------------------------------------------------------------------
set(ZLG_CANFD_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/usbcanfd_libusb_x64_1.0.12_251029")
set(ZLG_CANFD_LIB_PATH "${ZLG_CANFD_VENDOR_DIR}/libusbcanfd.so.1.0.12")
set(ZLG_ZUDS_LIB_PATH   "${ZLG_CANFD_VENDOR_DIR}/libzuds.so")

foreach(_zlg_lib IN ITEMS ${ZLG_CANFD_LIB_PATH} ${ZLG_ZUDS_LIB_PATH})
  if(NOT EXISTS ${_zlg_lib})
    message(FATAL_ERROR "ZLG USB-CAN FD library not found: ${_zlg_lib}. Please place vendor package under ${ZLG_CANFD_VENDOR_DIR}.")
  endif()
endforeach()

add_library(zlg_usbcanfd SHARED IMPORTED GLOBAL)
set_target_properties(zlg_usbcanfd PROPERTIES
  IMPORTED_LOCATION ${ZLG_CANFD_LIB_PATH}
)

add_library(zlg_zuds SHARED IMPORTED GLOBAL)
set_target_properties(zlg_zuds PROPERTIES
  IMPORTED_LOCATION ${ZLG_ZUDS_LIB_PATH}
)

# -----------------------------------------------------------------------------
# ROS2 包依赖查找
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)

# -----------------------------------------------------------------------------
# 创建 BrainCo 机械手驱动库
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/brainco_hand_api.cpp
  src/brainco_hand_hardware.cpp
)

# -----------------------------------------------------------------------------
# 设置编译特性
# C99 和 C++17 标准
# -----------------------------------------------------------------------------
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

# -----------------------------------------------------------------------------
# 设置包含目录
# BUILD_INTERFACE: 构建时包含目录
# INSTALL_INTERFACE: 安装后包含目录
# -----------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# 设置 ROS2 包依赖
# ament_target_dependencies 自动处理依赖的包含目录和链接库
# -----------------------------------------------------------------------------
ament_target_dependencies(
  ${PROJECT_NAME}
  "pluginlib"
  "rclcpp"
  "rclcpp_lifecycle"
  "hardware_interface"
)

# -----------------------------------------------------------------------------
# 链接外部依赖
# 将 BrainCo Stark SDK 链接到主项目
# -----------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}
  brainco_stark_sdk
  zlg_usbcanfd
  zlg_zuds
)

# 运行时确保可定位到 vendor 中的共享库（构建阶段与安装后）
set(_brainco_build_rpath
  "${BRAINCO_STARK_SDK_LIB_DIR}"
  "${ZLG_CANFD_VENDOR_DIR}"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${_brainco_build_rpath}"
  INSTALL_RPATH "\$ORIGIN/.."
)

# -----------------------------------------------------------------------------
# 插件系统配置
# 导出硬件接口插件描述文件，使 ROS2 能发现并加载此硬件接口插件
# -----------------------------------------------------------------------------
pluginlib_export_plugin_description_file(hardware_interface brainco_hand_driver_plugins.xml)

# -----------------------------------------------------------------------------
# 安装配置
# -----------------------------------------------------------------------------
# 安装头文件目录
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装随包分发的第三方共享库（Stark SDK + ZLG CANFD）
set(BRAINCO_VENDOR_SHARED_LIBS
  ${BRAINCO_STARK_SDK_LIB_PATH}
  ${ZLG_CANFD_LIB_PATH}
  ${ZLG_ZUDS_LIB_PATH}
)

install(
  FILES ${BRAINCO_VENDOR_SHARED_LIBS}
  DESTINATION lib
)

# 安装主项目库和可执行文件
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}        # 导出目标用于其他包依赖
  ARCHIVE DESTINATION lib             # 静态库
  LIBRARY DESTINATION lib             # 动态库
  RUNTIME DESTINATION bin             # 可执行文件
)

# 安装插件配置文件
install(FILES
  brainco_hand_driver_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 安装运行时所需的文件和目录
install(DIRECTORY
  launch      # 启动文件
  config      # 配置文件
  scripts     # 脚本文件
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS  # 保持原始文件权限
)

# -----------------------------------------------------------------------------
# 包导出配置
# 让其他 ROS2 包能够找到并使用此包
# -----------------------------------------------------------------------------
# 导出包含目录
ament_export_include_directories(
  include
)

# 导出库
ament_export_libraries(
  ${PROJECT_NAME}
)

# 导出 CMake 目标
ament_export_targets(
  export_${PROJECT_NAME}
)

# 导出依赖关系
ament_export_dependencies(
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)

# -----------------------------------------------------------------------------
# 测试配置
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 自动查找和配置代码质量检查工具（如 cppcheck, cpplint 等）
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------------------------------------------------------------
# 包声明结束
# 必须是 ROS2 CMake 包的最后一个命令
# -----------------------------------------------------------------------------
ament_package()
