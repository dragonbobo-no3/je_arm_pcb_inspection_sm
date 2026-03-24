#pragma once

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <je_software/msg/end_effector_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "gripper_command_loader.hpp"

namespace je_arm_pcb_inspection_sm
{
namespace utils
{

struct TestGripperConfig
{
  // 夹爪控制参数
  int mode{je_software::msg::EndEffectorCommand::MODE_POSITION};
  double position{0.0};
  int preset{0};
  std::string command;
  double torque{0.0};
  bool leftValid{true};
  bool rightValid{false};
  std::string topic{"/end_effector_cmd_lr"};
  
  // 执行参数
  double delayBeforeSec{0.0};
  double timeoutSec{5.0};
  std::string logLevel{"INFO"};
  
  std::string sourcePath;
  bool loaded{false};
};

inline TestGripperConfig loadTestGripperConfig()
{
  TestGripperConfig cfg;

  try
  {
    cfg.sourcePath =
      ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm") +
      "/config/test_gripper.yaml";

    const YAML::Node root = YAML::LoadFile(cfg.sourcePath);
    const YAML::Node testGripper = root["test_gripper"];

    if (testGripper)
    {
      // 夹爪控制参数
      cfg.mode = parseGripperMode(testGripper["mode"], cfg.mode);
      
      if (testGripper["position"])
      {
        cfg.position = testGripper["position"].as<double>();
      }
      
      if (testGripper["preset"])
      {
        cfg.preset = testGripper["preset"].as<int>();
      }

      if (testGripper["command"])
      {
        cfg.command = testGripper["command"].as<std::string>();
      }

      if (testGripper["torque"])
      {
        cfg.torque = testGripper["torque"].as<double>();
      }
      
      if (testGripper["left_valid"])
      {
        cfg.leftValid = testGripper["left_valid"].as<bool>();
      }
      
      if (testGripper["right_valid"])
      {
        cfg.rightValid = testGripper["right_valid"].as<bool>();
      }
      
      if (testGripper["topic"])
      {
        cfg.topic = testGripper["topic"].as<std::string>();
      }
      
      // 执行参数
      if (testGripper["delay_before_sec"])
      {
        cfg.delayBeforeSec = testGripper["delay_before_sec"].as<double>();
      }
      
      if (testGripper["timeout_sec"])
      {
        cfg.timeoutSec = testGripper["timeout_sec"].as<double>();
      }
      
      if (testGripper["log_level"])
      {
        cfg.logLevel = testGripper["log_level"].as<std::string>();
      }

      cfg.loaded = true;
    }
    else
    {
      RCLCPP_WARN(
        rclcpp::get_logger("TestGripperConfigLoader"),
        "test_gripper config section not found in %s, using defaults",
        cfg.sourcePath.c_str());
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("TestGripperConfigLoader"),
      "Failed to load test_gripper config from %s: %s",
      cfg.sourcePath.c_str(),
      e.what());
  }

  return cfg;
}

}  // namespace utils
}  // namespace je_arm_pcb_inspection_sm

