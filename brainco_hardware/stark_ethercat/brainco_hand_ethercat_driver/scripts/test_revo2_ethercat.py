#!/usr/bin/env python3
"""
Revo2 EtherCAT 节点测试脚本
用于测试ROS2环境、6指电机同时/单指控制等功能

"""

import os
import sys
import time
import argparse
import subprocess
import threading
from typing import List, Dict, Optional, Tuple
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Revo2EtherCATTester(Node):
    """Revo2 EtherCAT 测试节点"""

    def __init__(self, hand_type: str = None):
        super().__init__('revo2_ethercat_tester')

        # 检测或设置手类型
        self.hand_type = hand_type if hand_type else self._detect_hand_type()

        if self.hand_type not in ['left', 'right']:
            raise ValueError(f"无效的手类型: {self.hand_type}，必须是 'left' 或 'right'")

        # 根据手类型设置关节名称
        self.joint_names = self._get_joint_names_for_hand(self.hand_type)

        # 设置控制器名称
        self.controller_name = f'{self.hand_type}_revo2_hand_controller'
        self.trajectory_topic = f'/{self.controller_name}/joint_trajectory'
        self.action_name = f'/{self.controller_name}/follow_joint_trajectory'

        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 发布器
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.trajectory_topic,
            qos_profile
        )

        # 订阅器
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )

        # Action客户端
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name
        )

        # 状态存储
        self.current_joint_states = {}
        self.test_running = False
        self.state_lock = threading.Lock()

        self.get_logger().info(f"Revo2 EtherCAT Tester 初始化完成，手类型: {self.hand_type}")
        self.get_logger().info(f"控制器名称: {self.controller_name}")
        self.get_logger().info(f"关节名称: {self.joint_names}")

    def _detect_hand_type(self) -> str:
        """检测当前系统的手类型"""
        try:
            # 检查控制器列表
            result = subprocess.run(['ros2', 'control', 'list_controllers'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                controllers_output = result.stdout
                if 'right_revo2_hand_controller' in controllers_output:
                    return 'right'
                elif 'left_revo2_hand_controller' in controllers_output:
                    return 'left'

            # 检查话题列表
            result = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                topics_output = result.stdout
                if '/right_revo2_hand_controller/' in topics_output:
                    return 'right'
                elif '/left_revo2_hand_controller/' in topics_output:
                    return 'left'

            # 检查硬件组件
            result = subprocess.run(['ros2', 'control', 'list_hardware_interfaces'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                interfaces_output = result.stdout
                if 'Revo2RightEthercatSystem' in interfaces_output:
                    return 'right'
                elif 'Revo2LeftEthercatSystem' in interfaces_output:
                    return 'left'

        except Exception as e:
            self.get_logger().warn(f"自动检测手类型失败: {e}")

        # 默认返回右手
        self.get_logger().warn("无法自动检测手类型，默认使用右手配置")
        return 'right'

    def _get_joint_names_for_hand(self, hand_type: str) -> List[str]:
        """根据手类型获取关节名称"""
        if hand_type == 'left':
            return [
                'left_thumb_proximal_joint',
                'left_thumb_metacarpal_joint',
                'left_index_proximal_joint',
                'left_middle_proximal_joint',
                'left_ring_proximal_joint',
                'left_pinky_proximal_joint'
            ]
        elif hand_type == 'right':
            return [
                'right_thumb_proximal_joint',
                'right_thumb_metacarpal_joint',
                'right_index_proximal_joint',
                'right_middle_proximal_joint',
                'right_ring_proximal_joint',
                'right_pinky_proximal_joint'
            ]
        else:
            raise ValueError(f"不支持的手类型: {hand_type}")
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        with self.state_lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.current_joint_states[name] = {
                        'position': msg.position[i] if i < len(msg.position) else 0.0,
                        'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                        'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                    }
    
    def wait_for_action_server(self, timeout: float = 10.0) -> bool:
        """等待Action服务器就绪"""
        self.get_logger().info("等待Action服务器...")
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f"Action服务器未在{timeout}秒内就绪")
            return False
        self.get_logger().info("Action服务器已就绪")
        return True
    
    def create_trajectory_point(self, positions: List[float], time_from_start: float) -> JointTrajectoryPoint:
        """创建轨迹点"""
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
        return point
    
    def send_trajectory(self, positions: List[float], duration: float = 2.0) -> bool:
        """发送轨迹命令"""
        if len(positions) != len(self.joint_names):
            self.get_logger().error(f"位置数量({len(positions)})与关节数量({len(self.joint_names)})不匹配")
            return False
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points = [self.create_trajectory_point(positions, duration)]
        
        self.get_logger().info(f"发送轨迹: {positions}")
        self.trajectory_pub.publish(trajectory)
        return True
    
    def send_action_trajectory(self, positions: List[float], duration: float = 2.0) -> bool:
        """通过Action发送轨迹"""
        if not self.wait_for_action_server():
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [self.create_trajectory_point(positions, duration)]
        
        self.get_logger().info(f"发送Action轨迹: {positions}")
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Action目标被拒绝")
            return False
        
        self.get_logger().info("Action目标已接受，等待完成...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f"Action完成，结果: {result}")
        return True
    
    def print_current_states(self):
        """打印当前关节状态"""
        self.get_logger().info("=== 当前关节状态 ===")
        with self.state_lock:
            states_copy = dict(self.current_joint_states)
        for name in self.joint_names:
            if name in states_copy:
                state = states_copy[name]
                self.get_logger().info(
                    f"{name}: pos={state['position']:.4f}, "
                    f"vel={state['velocity']:.4f}, "
                    f"eff={state['effort']:.4f}"
                )
            else:
                self.get_logger().warn(f"{name}: 无状态数据")
    
    def all_fingers_test(self, amplitude: float = 0.5) -> bool:
        """全指同时控制测试"""
        self.get_logger().info(f"全指同时控制: 幅度={amplitude}")
        positions = [amplitude] * len(self.joint_names)
        return self.send_trajectory(positions, 0.5)
    
    def home_position_test(self) -> bool:
        """回到初始位置测试"""
        self.get_logger().info("执行归零测试")
        home_positions = [0.0] * len(self.joint_names)
        return self.send_trajectory(home_positions, 2.0)
    
    def individual_joint_test(self, joint_index: int, amplitude: float = 0.5) -> bool:
        """单个关节测试"""
        if joint_index < 0 or joint_index >= len(self.joint_names):
            self.get_logger().error(f"关节索引{joint_index}超出范围")
            return False
        
        self.get_logger().info(f"测试关节: {self.joint_names[joint_index]}")
        
        # 创建只有目标关节运动的轨迹
        positions = [0.0] * len(self.joint_names)
        positions[joint_index] = amplitude
        
        return self.send_trajectory(positions, 0.5)


class ROS2EnvironmentChecker:
    """ROS2环境检查器"""
    
    @staticmethod
    def check_ros2_environment() -> Dict[str, bool]:
        """检查ROS2环境"""
        results = {}
        
        # # 检查ROS2是否安装
        # try:
        #     result = subprocess.run(['ros2', '--version'], 
        #                           capture_output=True, text=True, timeout=5)
        #     results['ros2_installed'] = result.returncode == 0
        #     if results['ros2_installed']:
        #         print(f"✓ ROS2版本: {result.stdout.strip()}")
        # except Exception as e:
        #     results['ros2_installed'] = False
        #     print(f"✗ ROS2未安装: {e}")
        
        # 检查环境变量
        results['ros_domain_id'] = 'ROS_DOMAIN_ID' in os.environ
        results['ros_distro'] = 'ROS_DISTRO' in os.environ
        
        if results['ros_distro']:
            print(f"✓ ROS_DISTRO: {os.environ['ROS_DISTRO']}")
            results['ros2_installed'] = True
        if results['ros_domain_id']:
            print(f"✓ ROS_DOMAIN_ID: {os.environ['ROS_DOMAIN_ID']}")
        
        return results
    
    @staticmethod
    def check_nodes_and_topics() -> Dict[str, List[str]]:
        """检查节点和话题"""
        results = {'nodes': [], 'topics': []}
        
        try:
            # 检查节点
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                results['nodes'] = result.stdout.strip().split('\n')
                print(f"✓ 发现{len(results['nodes'])}个节点")
            
            # 检查话题
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                results['topics'] = result.stdout.strip().split('\n')
                print(f"✓ 发现{len(results['topics'])}个话题")
                
        except Exception as e:
            print(f"✗ 检查节点/话题失败: {e}")
        
        return results
    
    @staticmethod
    def check_controllers(max_retries: int = 3, retry_delay: float = 1.0) -> Dict[str, any]:
        """检查控制器状态（带重试机制）"""
        results = {}

        try:
            # 检查控制器列表
            result = subprocess.run(['ros2', 'control', 'list_controllers'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                results['controllers'] = result.stdout.strip()
                print("✓ 控制器状态:")
                print(results['controllers'])

                # 检测手类型
                controllers_output = results['controllers']
                if 'right_revo2_hand_controller' in controllers_output:
                    results['detected_hand_type'] = 'right'
                    print("✓ 检测到右手控制器")
                elif 'left_revo2_hand_controller' in controllers_output:
                    results['detected_hand_type'] = 'left'
                    print("✓ 检测到左手控制器")
                else:
                    results['detected_hand_type'] = None
                    print("⚠ 未检测到 Revo2 手控制器")

            # 检查硬件接口（带重试机制）
            hardware_component_detected = False
            for attempt in range(max_retries):
            result = subprocess.run(['ros2', 'control', 'list_hardware_interfaces'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                results['hardware_interfaces'] = result.stdout.strip()
                    
                    # 只在第一次或最后一次尝试时打印完整信息
                    if attempt == 0 or attempt == max_retries - 1:
                print("✓ 硬件接口:")
                print(results['hardware_interfaces'])

                # 检查硬件组件类型
                interfaces_output = results['hardware_interfaces']
                    
                    # 检查组件名称
                if 'Revo2RightEthercatSystem' in interfaces_output:
                    results['hardware_component_type'] = 'right'
                    print("✓ 检测到右手硬件组件")
                        hardware_component_detected = True
                        break
                elif 'Revo2LeftEthercatSystem' in interfaces_output:
                    results['hardware_component_type'] = 'left'
                    print("✓ 检测到左手硬件组件")
                        hardware_component_detected = True
                        break
                else:
                        # 如果没找到组件名称，检查是否有可用的接口（作为备用检测）
                        # 检查是否有 command interfaces 或 state interfaces
                        if 'command interfaces' in interfaces_output.lower() or 'state interfaces' in interfaces_output.lower():
                            # 检查是否有 Revo2 相关的关节接口
                            if any(joint in interfaces_output for joint in [
                                'right_thumb_proximal_joint', 'left_thumb_proximal_joint',
                                'right_index_proximal_joint', 'left_index_proximal_joint'
                            ]):
                                # 根据关节名称推断手类型
                                if 'right_' in interfaces_output and 'right_thumb_proximal_joint' in interfaces_output:
                                    results['hardware_component_type'] = 'right'
                                    print("✓ 通过接口检测到右手硬件组件")
                                    hardware_component_detected = True
                                    break
                                elif 'left_' in interfaces_output and 'left_thumb_proximal_joint' in interfaces_output:
                                    results['hardware_component_type'] = 'left'
                                    print("✓ 通过接口检测到左手硬件组件")
                                    hardware_component_detected = True
                                    break
                
                # 如果还没检测到，等待后重试
                if not hardware_component_detected and attempt < max_retries - 1:
                    time.sleep(retry_delay)
            
            # 如果所有重试都失败
            if not hardware_component_detected:
                    results['hardware_component_type'] = None
                # 检查是否有接口可用（即使没有组件名称）
                if 'hardware_interfaces' in results:
                    interfaces_output = results['hardware_interfaces']
                    # 如果接口列表不为空，说明硬件可能在工作，只是组件名称未显示
                    if interfaces_output and len(interfaces_output.strip()) > 0:
                        print("ℹ 硬件接口可用，但未检测到组件名称（可能正常，硬件功能正常）")
                    else:
                        print("⚠ 未检测到 Revo2 硬件组件和接口")

        except Exception as e:
            print(f"✗ 检查控制器失败: {e}")

        return results


class TestMenu:
    """测试菜单系统"""

    def __init__(self, hand_type: str = None):
        self.tester = None
        self.executor = None
        self.executor_thread = None
        self.forced_hand_type = hand_type
    
    def setup_ros2(self):
        """设置ROS2环境"""
        print("\n=== 设置ROS2环境 ===")
        
        # 检查环境
        env_check = ROS2EnvironmentChecker.check_ros2_environment()
        if not env_check['ros2_installed']:
            print("❌ ROS2未正确安装，请先安装ROS2")
            return False
        
        # 初始化ROS2
        try:
            rclpy.init()
            self.tester = Revo2EtherCATTester(hand_type=self.forced_hand_type)
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.tester)
            # 启动执行器后台线程以驱动订阅回调
            self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()
            print("✓ ROS2节点与执行器初始化成功")
            print(f"✓ 使用手类型: {self.tester.hand_type}")
            return True
        except Exception as e:
            print(f"❌ ROS2初始化失败: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        try:
            if self.executor:
                # 停止执行器spin
                self.executor.shutdown()
            if self.executor_thread and self.executor_thread.is_alive():
                self.executor_thread.join(timeout=1.0)
        except Exception:
            pass
        finally:
            if self.tester:
                try:
                    self.tester.destroy_node()
                except Exception:
                    pass
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
        print("✓ 资源清理完成")
    
    def run_environment_test(self):
        """运行环境测试"""
        print("\n=== ROS2环境测试 ===")

        # 检查ROS2环境
        env_results = ROS2EnvironmentChecker.check_ros2_environment()

        # 检查节点和话题
        node_topic_results = ROS2EnvironmentChecker.check_nodes_and_topics()

        # 等待一段时间让节点启动（在检查控制器之前）
        print("等待节点启动...")
        time.sleep(2)  # 增加等待时间到2秒

        # 检查控制器（带重试机制）
        controller_results = ROS2EnvironmentChecker.check_controllers(max_retries=3, retry_delay=1.0)

        # 检测手类型
        detected_hand_type = controller_results.get('detected_hand_type')

        if detected_hand_type:
            print(f"\n✓ 检测到手类型: {detected_hand_type}")
            # 设置测试器手类型（如果尚未初始化）
            if hasattr(self, 'tester') and self.tester and self.tester.hand_type != detected_hand_type:
                print(f"⚠ 测试器手类型 ({self.tester.hand_type}) 与检测到的手类型 ({detected_hand_type}) 不匹配")
                print("建议重新初始化测试器以匹配当前系统")
        else:
            print("\n⚠ 未检测到有效的 Revo2 手系统")
            return False

        # 根据检测到的手类型设置关键话题
        hand_type = detected_hand_type
        key_topics = [
            '/joint_states',
            f'/{hand_type}_revo2_hand_controller/joint_trajectory',
            f'/{hand_type}_revo2_hand_controller/state'
        ]

        print(f"\n检查关键话题 ({hand_type} 手):")
        for topic in key_topics:
            if topic in node_topic_results['topics']:
                print(f"✓ {topic}")
            else:
                print(f"✗ {topic} (未找到)")

        # 检查手类型一致性
        controller_hand_type = controller_results.get('detected_hand_type')
        hardware_hand_type = controller_results.get('hardware_component_type')

        if controller_hand_type and hardware_hand_type:
            if controller_hand_type == hardware_hand_type:
                print(f"\n✓ 控制器和硬件组件手类型一致: {controller_hand_type}")
            else:
                print(f"\n⚠ 控制器手类型 ({controller_hand_type}) 与硬件组件手类型 ({hardware_hand_type}) 不匹配")
                return False

        return True
    
    def run_all_fingers_test(self):
        """运行全指同时控制测试"""
        print("\n=== 全指同时控制测试 ===")
        
        if not self.tester:
            print("❌ 测试器未初始化")
            return False
        
        # 显示当前状态
        self.tester.print_current_states()
        
        # 执行测试
        success = self.tester.all_fingers_test(0.8)
        time.sleep(2)  # 等待运动完成
        
        # 显示当前状态
        self.tester.print_current_states()
        if success:
            print("✓ 全指同时控制测试完成")
        else:
            print("❌ 全指同时控制测试失败")
        
        return success
    
    def run_individual_joint_test(self):
        """运行单关节测试"""
        print("\n=== 单关节测试 ===")
        
        if not self.tester:
            print("❌ 测试器未初始化")
            return False
        
        # 显示关节列表
        print("可用关节:")
        for i, name in enumerate(self.tester.joint_names):
            print(f"  {i}: {name}")
        
        # 测试每个关节
        for i in range(len(self.tester.joint_names)):
            print(f"\n测试关节 {i}: {self.tester.joint_names[i]}")
            self.tester.individual_joint_test(i, 0.8)
            time.sleep(2)  # 等待运动完成
        
        print("✓ 单关节测试完成")
        return True
    
    def run_home_test(self):
        """运行归零测试"""
        print("\n=== 归零测试 ===")
        
        if not self.tester:
            print("❌ 测试器未初始化")
            return False
        
        print("执行归零运动...")
        success = self.tester.home_position_test()
        
        if success:
            print("✓ 归零测试完成")
        else:
            print("❌ 归零测试失败")
        
        return success
    
    def run_show_states_test(self):
        """运行显示当前关节状态测试"""
        print("\n=== 显示当前关节状态 ===")
        
        if not self.tester:
            print("❌ 测试器未初始化")
            return False
        
        # 等待一小段时间确保有状态数据
        time.sleep(0.5)
        
        # 显示当前状态
        self.tester.print_current_states()
        print("✓ 关节状态显示完成")
        
        return True
    
    def show_menu(self):
        """显示主菜单"""
        print("\n" + "="*50)
        print("Revo2 EtherCAT 测试菜单")
        print("="*50)
        print("1. ROS2环境测试")
        print("2. 全指同时控制测试")
        print("3. 单关节测试")
        print("4. 归零测试")
        print("5. 显示当前关节状态")
        print("6. 运行所有测试")
        print("0. 退出")
        print("="*50)
    
    def run_all_tests(self):
        """运行所有测试"""
        print("\n=== 运行所有测试 ===")
        
        tests = [
            ("环境测试", self.run_environment_test),
            ("显示当前关节状态", self.run_show_states_test),
            ("归零测试", self.run_home_test),
            ("单关节测试", self.run_individual_joint_test),
            ("全指同时控制测试", self.run_all_fingers_test),
        ]
        
        results = {}
        for test_name, test_func in tests:
            print(f"\n--- 执行: {test_name} ---")
            try:
                results[test_name] = test_func()
            except Exception as e:
                print(f"❌ {test_name} 失败: {e}")
                results[test_name] = False
        
        # 显示结果摘要
        print("\n=== 测试结果摘要 ===")
        for test_name, success in results.items():
            status = "✓ 通过" if success else "❌ 失败"
            print(f"{test_name}: {status}")
        
        return results
    
    def run(self):
        """运行主循环"""
        try:
            while True:
                self.show_menu()
                choice = input("请选择测试项目 (0-6): ").strip()
                
                if choice == '0':
                    print("退出测试程序")
                    break
                elif choice == '1':
                    self.run_environment_test()
                elif choice == '2':
                    if not self.tester:
                        if not self.setup_ros2():
                            continue
                    self.run_all_fingers_test()
                elif choice == '3':
                    if not self.tester:
                        if not self.setup_ros2():
                            continue
                    self.run_individual_joint_test()
                elif choice == '4':
                    if not self.tester:
                        if not self.setup_ros2():
                            continue
                    self.run_home_test()
                elif choice == '5':
                    if not self.tester:
                        if not self.setup_ros2():
                            continue
                    self.tester.print_current_states()
                elif choice == '6':
                    if not self.tester:
                        if not self.setup_ros2():
                            continue
                    self.run_all_tests()
                else:
                    print("无效选择，请重新输入")
                
                input("\n按回车键继续...")
                
        except KeyboardInterrupt:
            print("\n\n程序被用户中断")
        finally:
            self.cleanup()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Revo2 EtherCAT 测试脚本')
    parser.add_argument('--test', type=str, choices=[
        'env', 'all_fingers', 'individual', 'home', 'all'
    ], help='直接运行指定测试')
    parser.add_argument('--amplitude', type=float, default=0.3,
                       help='运动幅度')
    parser.add_argument('--menu', action='store_true',
                       help='显示交互式菜单')
    parser.add_argument('--hand-type', type=str, choices=['left', 'right'],
                       help='强制指定手类型 (left 或 right)，默认自动检测')

    args = parser.parse_args()

    # 创建菜单系统
    menu = TestMenu(hand_type=args.hand_type)
    
    if args.menu or not args.test:
        # 交互式菜单模式
        menu.run()
    else:
        # 命令行模式
        try:
            if args.test == 'env':
                menu.run_environment_test()
            elif args.test in ['all_fingers', 'individual', 'home', 'all']:
                if not menu.setup_ros2():
                    sys.exit(1)
                
                if args.test == 'all_fingers':
                    menu.tester.all_fingers_test(args.amplitude)
                elif args.test == 'individual':
                    menu.run_individual_joint_test()
                elif args.test == 'home':
                    menu.run_home_test()
                elif args.test == 'all':
                    menu.run_all_tests()
        finally:
            menu.cleanup()

if __name__ == '__main__':
    main()
