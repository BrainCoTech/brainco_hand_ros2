// Copyright (c) 2026 BrainCo
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "brainco_hand_driver/revo2_pid_controller.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace revo2_driver
{
namespace
{
using Result = controller_interface::return_type;
using CallbackResult = controller_interface::CallbackReturn;

class IntegralControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    positions_.fill(0.5);
    commands_.fill(0.0);
    controller_ = std::make_unique<Revo2PidController>();
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.enable_rosout(false);
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    ASSERT_EQ(controller_->init("integral_controller_test", "", options), Result::OK);

    const std::vector<std::string> joints{
      "right_thumb_proximal_joint", "right_thumb_metacarpal_joint",
      "right_index_proximal_joint", "right_middle_proximal_joint",
      "right_ring_proximal_joint", "right_pinky_proximal_joint"};
    auto node = controller_->get_node();
    node->set_parameters({
      rclcpp::Parameter("joints", joints),
      rclcpp::Parameter("target_topic", "/integral_controller_test/target"),
      rclcpp::Parameter("target_timeout", 0.0),
      rclcpp::Parameter("target_filter.alpha", 1.0),
      rclcpp::Parameter("target_filter.fast_alpha", 1.0),
      rclcpp::Parameter("target_velocity_feedforward.gain", 0.0),
      rclcpp::Parameter("pd_velocity.velocity_kp", 0.0),
      rclcpp::Parameter("pd_velocity.velocity_kd", 0.0),
      rclcpp::Parameter("pd_velocity.velocity_ki", 1.0),
      rclcpp::Parameter("pd_velocity.integral_max", 0.2),
      rclcpp::Parameter("pd_velocity.velocity_max", 10.0),
      rclcpp::Parameter("pd_velocity.velocity_slew_rate", 100.0),
      rclcpp::Parameter("pd_velocity.velocity_deadband", 0.001),
      rclcpp::Parameter("pd_velocity.thumb_velocity_deadband", 0.001),
      rclcpp::Parameter("pd_velocity.ring_velocity_deadband", 0.001),
      rclcpp::Parameter("pd_velocity.thumb_velocity_kp_scale", 1.0),
      rclcpp::Parameter("pd_velocity.ring_velocity_kp_scale", 1.0),
      rclcpp::Parameter("pd_velocity.thumb_velocity_brake_zone", 0.0),
      rclcpp::Parameter("pd_velocity.four_finger_extension_velocity_scale", 1.0)});

    command_handles_.reserve(6);
    state_handles_.reserve(12);
    for (std::size_t i = 0; i < 6; ++i) {
      command_handles_.emplace_back(joints[i], "velocity", &commands_[i]);
      state_handles_.emplace_back(joints[i], "position", &positions_[i]);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      state_handles_.emplace_back(joints[i], "velocity", &velocities_[i]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (auto & handle : command_handles_) {
      commands.emplace_back(handle);
    }
    for (auto & handle : state_handles_) {
      states.emplace_back(handle);
    }
    controller_->assign_interfaces(std::move(commands), std::move(states));

    publisher_node_ = std::make_shared<rclcpp::Node>("integral_test_publisher", options);
    publisher_ = publisher_node_->create_publisher<sensor_msgs::msg::JointState>(
      "/integral_controller_test/target", 10);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node->get_node_base_interface());
    executor_->add_node(publisher_node_);
  }

  void TearDown() override
  {
    if (controller_) {
      controller_->on_deactivate(rclcpp_lifecycle::State{});
    }
    executor_.reset();
    publisher_.reset();
    publisher_node_.reset();
    controller_.reset();
  }

  void Configure(std::vector<rclcpp::Parameter> parameters = {})
  {
    controller_->get_node()->set_parameters(parameters);
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackResult::SUCCESS);
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}), CallbackResult::SUCCESS);
  }

  void Target(double index = 0.6, double middle = 0.5, double thumb = 0.5)
  {
    sensor_msgs::msg::JointState msg;
    msg.position = {thumb, 0.5, index, middle, 0.5, 0.5};
    publisher_->publish(msg);
    executor_->spin_some();
  }

  Result Step(double dt = 0.1)
  {
    return controller_->update(
      controller_->get_node()->get_clock()->now(), rclcpp::Duration::from_seconds(dt));
  }

  std::array<double, 6> positions_{};
  std::array<double, 6> velocities_{};
  std::array<double, 6> commands_{};
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
  std::unique_ptr<Revo2PidController> controller_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(IntegralControllerTest, ZeroGainPreservesProportionalOutput)
{
  Configure({rclcpp::Parameter("pd_velocity.velocity_ki", 0.0),
    rclcpp::Parameter("pd_velocity.velocity_kp", 2.0)});
  Target();
  for (int i = 0; i < 30; ++i) {
    ASSERT_EQ(Step(), Result::OK);
    EXPECT_NEAR(commands_[2], 0.2, 1e-10);
  }
}

TEST_F(IntegralControllerTest, IntegratesElapsedTimeAndIndependentJointErrors)
{
  Configure();
  Target(0.6, 0.4);
  ASSERT_EQ(Step(0.1), Result::OK);
  EXPECT_NEAR(commands_[2], 0.01, 1e-10);
  EXPECT_NEAR(commands_[3], -0.01, 1e-10);
  EXPECT_DOUBLE_EQ(commands_[0], 0.0);
  ASSERT_EQ(Step(0.2), Result::OK);
  EXPECT_NEAR(commands_[2], 0.03, 1e-10);
  EXPECT_NEAR(commands_[3], -0.03, 1e-10);
}

TEST_F(IntegralControllerTest, IntegralContributionIsBoundedInBothDirections)
{
  Configure();
  Target(0.6, 0.4);
  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(Step(), Result::OK);
  }
  EXPECT_NEAR(commands_[2], 0.2, 1e-10);
  EXPECT_NEAR(commands_[3], -0.2, 1e-10);
}

TEST_F(IntegralControllerTest, ReversalAndDeadbandClearStoredIntegral)
{
  Configure();
  Target();
  for (int i = 0; i < 5; ++i) {Step();}
  EXPECT_NEAR(commands_[2], 0.05, 1e-10);
  Target(0.4);
  Step();
  EXPECT_NEAR(commands_[2], -0.01, 1e-10);
  Target(0.5);
  Step();
  EXPECT_DOUBLE_EQ(commands_[2], 0.0);
  Target();
  Step();
  EXPECT_NEAR(commands_[2], 0.01, 1e-10);
}

TEST_F(IntegralControllerTest, VelocitySaturationDoesNotWindUp)
{
  Configure({rclcpp::Parameter("pd_velocity.velocity_kp", 1.0),
    rclcpp::Parameter("pd_velocity.velocity_max", 0.05)});
  Target();
  for (int i = 0; i < 100; ++i) {Step();}
  EXPECT_NEAR(commands_[2], 0.05, 1e-10);
  Target(0.51);
  Step();
  EXPECT_NEAR(commands_[2], 0.011, 1e-10);
}

TEST_F(IntegralControllerTest, SmallIntegralCanAccumulatePastOutputZeroEpsilon)
{
  Configure({rclcpp::Parameter("pd_velocity.velocity_ki", 0.1)});
  Target();
  for (int i = 0; i < 10; ++i) {Step(0.01);}
  EXPECT_DOUBLE_EQ(commands_[2], 0.0);
  for (int i = 0; i < 21; ++i) {Step(0.01);}
  EXPECT_NEAR(commands_[2], 0.0031, 1e-10);
}

TEST_F(IntegralControllerTest, SlewSaturationDoesNotWindUp)
{
  Configure({rclcpp::Parameter("pd_velocity.velocity_slew_rate", 0.005)});
  Target();
  for (int i = 0; i < 5; ++i) {Step();}
  EXPECT_NEAR(commands_[2], 0.025, 1e-10);
  Target(0.51);
  Step();
  EXPECT_NEAR(commands_[2], 0.021, 1e-10);
}

TEST_F(IntegralControllerTest, ThumbBrakeSaturationDoesNotWindUp)
{
  Configure({rclcpp::Parameter("pd_velocity.thumb_velocity_brake_zone", 0.2),
    rclcpp::Parameter("pd_velocity.thumb_velocity_brake_max", 0.02)});
  Target(0.5, 0.5, 0.6);
  for (int i = 0; i < 5; ++i) {Step();}
  EXPECT_NEAR(commands_[0], 0.01, 1e-10);
  positions_[0] = 0.2;
  Step();
  EXPECT_NEAR(commands_[0], 0.05, 1e-10);
}

TEST_F(IntegralControllerTest, ZeroIntegralLimitDisablesIntegration)
{
  Configure({rclcpp::Parameter("pd_velocity.integral_max", 0.0),
    rclcpp::Parameter("pd_velocity.velocity_kp", 2.0)});
  Target();
  for (int i = 0; i < 30; ++i) {Step();}
  EXPECT_NEAR(commands_[2], 0.2, 1e-10);
}

TEST_F(IntegralControllerTest, TimeoutClearsIntegralBeforeNewTarget)
{
  Configure({rclcpp::Parameter("target_timeout", 0.3)});
  Target();
  for (int i = 0; i < 5; ++i) {Step();}
  EXPECT_NEAR(commands_[2], 0.05, 1e-10);
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  Step();
  EXPECT_DOUBLE_EQ(commands_[2], 0.0);
  Target();
  Step();
  EXPECT_NEAR(commands_[2], 0.01, 1e-10);
}

TEST_F(IntegralControllerTest, ReactivationClearsIntegralAndRequiresNewTarget)
{
  Configure();
  Target();
  for (int i = 0; i < 5; ++i) {Step();}
  controller_->on_deactivate(rclcpp_lifecycle::State{});
  EXPECT_DOUBLE_EQ(commands_[2], 0.0);
  controller_->on_activate(rclcpp_lifecycle::State{});
  Step();
  EXPECT_DOUBLE_EQ(commands_[2], 0.0);
  Target();
  Step();
  EXPECT_NEAR(commands_[2], 0.01, 1e-10);
}

TEST_F(IntegralControllerTest, InvalidFeedbackClearsIntegralAndReturnsError)
{
  Configure();
  Target();
  for (int i = 0; i < 5; ++i) {Step();}
  positions_[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(Step(), Result::ERROR);
  for (double value : commands_) {EXPECT_DOUBLE_EQ(value, 0.0);}
  positions_[2] = 0.5;
  Step();
  EXPECT_NEAR(commands_[2], 0.01, 1e-10);
}

TEST_F(IntegralControllerTest, InvalidIntegralParametersRejectConfiguration)
{
  for (const auto & parameter : {
      rclcpp::Parameter("pd_velocity.velocity_ki", -0.1),
      rclcpp::Parameter("pd_velocity.velocity_ki", std::numeric_limits<double>::infinity())})
  {
    controller_->get_node()->set_parameter(parameter);
    EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackResult::ERROR);
  }
  controller_->get_node()->set_parameters({
    rclcpp::Parameter("pd_velocity.velocity_ki", 0.0),
    rclcpp::Parameter("pd_velocity.integral_max", -0.1)});
  EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackResult::ERROR);
}

}  // namespace
}  // namespace revo2_driver
