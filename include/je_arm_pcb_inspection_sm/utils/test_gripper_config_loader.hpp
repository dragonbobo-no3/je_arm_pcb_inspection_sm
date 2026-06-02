#pragma once

#include <vector>
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

struct TestGripperStep
{
  std::string name;
  int mode{je_software::msg::EndEffectorCommand::MODE_POSITION};
  double position{0.0};
  int preset{0};
  std::string command;
  double torque{0.0};
  bool leftValid{true};
  bool rightValid{false};
  double minDurationSec{1.0};
  double timeoutSec{5.0};
};

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
  bool waitForFeedback{false};
  std::string topic{"/end_effector_cmd_lr"};
  std::string feedbackTopic{"/joint_states_double_arm"};
  double positionTolerance{0.03};
  
  // 执行参数
  double delayBeforeSec{0.0};
  double timeoutSec{5.0};
  double stepHoldSec{1.0};
  std::string logLevel{"INFO"};
  std::vector<TestGripperStep> steps;
  
  std::string sourcePath;
  bool loaded{false};
};

inline TestGripperStep makeTestGripperStepFromConfig(const TestGripperConfig & cfg)
{
  TestGripperStep step;
  step.name = "single_step";
  step.mode = cfg.mode;
  step.position = cfg.position;
  step.preset = cfg.preset;
  step.command = cfg.command;
  step.torque = cfg.torque;
  step.leftValid = cfg.leftValid;
  step.rightValid = cfg.rightValid;
  step.minDurationSec = cfg.stepHoldSec;
  step.timeoutSec = cfg.timeoutSec;
  return step;
}

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

      if (testGripper["wait_for_feedback"])
      {
        cfg.waitForFeedback = testGripper["wait_for_feedback"].as<bool>();
      }
      
      if (testGripper["topic"])
      {
        cfg.topic = testGripper["topic"].as<std::string>();
      }

      if (testGripper["feedback_topic"])
      {
        cfg.feedbackTopic = testGripper["feedback_topic"].as<std::string>();
      }

      if (testGripper["position_tolerance"])
      {
        cfg.positionTolerance = testGripper["position_tolerance"].as<double>();
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

      if (testGripper["step_hold_sec"])
      {
        cfg.stepHoldSec = testGripper["step_hold_sec"].as<double>();
      }
      
      if (testGripper["log_level"])
      {
        cfg.logLevel = testGripper["log_level"].as<std::string>();
      }

      const YAML::Node steps = testGripper["steps"];
      if (steps && steps.IsSequence())
      {
        for (const auto & stepNode : steps)
        {
          TestGripperStep step = makeTestGripperStepFromConfig(cfg);

          if (stepNode["name"])
          {
            step.name = stepNode["name"].as<std::string>();
          }
          step.mode = parseGripperMode(stepNode["mode"], step.mode);

          if (stepNode["position"])
          {
            step.position = stepNode["position"].as<double>();
          }
          if (stepNode["preset"])
          {
            step.preset = stepNode["preset"].as<int>();
          }
          if (stepNode["command"])
          {
            step.command = stepNode["command"].as<std::string>();
          }
          if (stepNode["torque"])
          {
            step.torque = stepNode["torque"].as<double>();
          }
          if (stepNode["left_valid"])
          {
            step.leftValid = stepNode["left_valid"].as<bool>();
          }
          if (stepNode["right_valid"])
          {
            step.rightValid = stepNode["right_valid"].as<bool>();
          }
          if (stepNode["timeout_sec"])
          {
            step.timeoutSec = stepNode["timeout_sec"].as<double>();
          }
          if (stepNode["min_duration_sec"])
          {
            step.minDurationSec = stepNode["min_duration_sec"].as<double>();
          }

          cfg.steps.push_back(step);
        }
      }

      if (cfg.steps.empty())
      {
        cfg.steps.push_back(makeTestGripperStepFromConfig(cfg));
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

