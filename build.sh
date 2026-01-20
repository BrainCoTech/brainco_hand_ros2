#!/bin/bash
# =============================================================================
# BrainCo Hand ROS2 编译脚本
# =============================================================================
# 支持选择性编译 CAN FD 和 EtherCAT 功能
#
# 用法:
#   ./build.sh                          # 默认编译（不启用 CAN FD 和 EtherCAT）
#   ./build.sh --canfd                  # 启用 CAN FD 支持
#   ./build.sh --ethercat               # 启用 EtherCAT 支持
#   ./build.sh --canfd --ethercat       # 同时启用 CAN FD 和 EtherCAT
#   ./build.sh --release                # Release 模式编译
#   ./build.sh --release --canfd --ethercat  # Release 模式，启用所有功能
#
# 或者使用 colcon 命令直接编译:
#   colcon build --symlink-install --cmake-args -DENABLE_CANFD=ON
#   colcon build --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver
# =============================================================================

set -e  # 遇到错误立即退出

# 默认选项（默认都禁用）
ENABLE_CANFD=OFF
ENABLE_ETHERCAT=OFF
BUILD_TYPE=""
PACKAGES_IGNORE=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --canfd)
            ENABLE_CANFD=ON
            shift
            ;;
        --ethercat)
            ENABLE_ETHERCAT=ON
            shift
            ;;
        --release)
            BUILD_TYPE="-DCMAKE_BUILD_TYPE=Release"
            shift
            ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --canfd          启用 CAN FD 支持（默认禁用）"
            echo "  --ethercat       启用 EtherCAT 支持（默认禁用）"
            echo "  --release        使用 Release 模式编译"
            echo "  --help, -h       显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                                    # 默认编译（不启用 CAN FD 和 EtherCAT）"
            echo "  $0 --canfd                           # 启用 CAN FD 支持"
            echo "  $0 --ethercat                        # 启用 EtherCAT 支持"
            echo "  $0 --canfd --ethercat                # 同时启用 CAN FD 和 EtherCAT"
            echo "  $0 --release --canfd                 # Release 模式，启用 CAN FD"
            echo "  $0 --release --canfd --ethercat     # Release 模式，启用所有功能"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 构建 CMake 参数
CMAKE_ARGS=""
if [ "$ENABLE_CANFD" = "ON" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DENABLE_CANFD=ON"
    echo "✓ CAN FD 支持: 启用"
else
    echo "✓ CAN FD 支持: 禁用"
fi

if [ -n "$BUILD_TYPE" ]; then
    CMAKE_ARGS="$CMAKE_ARGS $BUILD_TYPE"
    echo "✓ 编译模式: Release"
else
    echo "✓ 编译模式: 默认"
fi

# 构建包忽略列表（如果禁用 EtherCAT）
if [ "$ENABLE_ETHERCAT" = "OFF" ]; then
    PACKAGES_IGNORE="--packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver"
    echo "✓ EtherCAT 支持: 禁用（将跳过 EtherCAT 相关包）"
else
    echo "✓ EtherCAT 支持: 启用"
fi

# 显示编译配置
echo ""
echo "=========================================="
echo "编译配置"
echo "=========================================="
echo "CAN FD:    $ENABLE_CANFD"
echo "EtherCAT:  $ENABLE_ETHERCAT"
if [ -n "$BUILD_TYPE" ]; then
    echo "模式:      Release"
fi
echo "=========================================="
echo ""

# 执行编译
# 构建完整的编译命令
BUILD_CMD="colcon build --symlink-install"

# 添加 CMake 参数
if [ -n "$CMAKE_ARGS" ]; then
    BUILD_CMD="$BUILD_CMD --cmake-args $CMAKE_ARGS"
fi

# 添加包忽略列表（如果禁用 EtherCAT）
if [ -n "$PACKAGES_IGNORE" ]; then
    BUILD_CMD="$BUILD_CMD $PACKAGES_IGNORE"
fi

# 执行编译命令
eval $BUILD_CMD

echo ""
echo "=========================================="
echo "编译完成！"
echo "=========================================="
echo "请运行以下命令激活工作空间:"
echo "  source install/setup.bash"
echo ""
