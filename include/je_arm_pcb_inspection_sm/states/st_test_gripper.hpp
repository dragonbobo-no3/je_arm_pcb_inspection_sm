#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

#include <cl_moveit2z/client_behaviors/cb_ctrl_gripper.hpp>
#include <je_software/msg/end_effector_command_lr.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/gripper_command_loader.hpp"
#include "je_arm_pcb_inspection_sm/utils/test_gripper_config_loader.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StIdle;
struct StPause;

/// TEST_GRIPPER 状态：测试夹爪功能
/// 从 config/test_gripper.yaml 直接加载夹爪参数（模式、位置、预设等）
/// 无需重新编译，修改 YAML 文件后立即生效
struct StTestGripper : smacc2::SmaccState<StTestGripper, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbCtrlGripper, OrGripper>, StIdle>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbCtrlGripper, OrGripper>, StPause>,
    smacc2::Transition<EvGripperClosed, StIdle>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    // 使用 YAML 默认值配置，实际参数在 onEntry 时从 test_gripper.yaml 读取
    configure_orthogonal<OrGripper, cl_moveit2z::CbCtrlGripper>(
      je_software::msg::EndEffectorCommand::MODE_POSITION,  // 默认模式
      0.0,                                                   // 默认位置（关闭）
      0,                                                     // 默认 preset
      true,                                                  // 默认左手有效
      false,                                                 // 默认右手无效
      "/end_effector_cmd_lr");                             // 默认 topic
  }

  void onEntry()
  {
    // 从 config/test_gripper.yaml 加载直接的夹爪参数
    const auto cfg = je_arm_pcb_inspection_sm::utils::loadTestGripperConfig();
    
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER TEST_GRIPPER - config loaded from test_gripper.yaml | "
      "mode=%d, position=%.2f, preset=%d, command=%s, torque=%.2f, left=%d, right=%d, topic='%s', timeout=%.2fs [bypass: 'n']",
      log_utils::bjtNowString().c_str(),
      cfg.mode,
      cfg.position,
      cfg.preset,
      cfg.command.c_str(),
      cfg.torque,
      cfg.leftValid,
      cfg.rightValid,
      cfg.topic.c_str(),
      cfg.timeoutSec);

    // 如果有延迟，先等待
    if (cfg.delayBeforeSec > 0.0)
    {
      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] Waiting %.2fs before gripper execution",
        log_utils::bjtNowString().c_str(),
        cfg.delayBeforeSec);
      std::this_thread::sleep_for(std::chrono::duration<double>(cfg.delayBeforeSec));
    }

    try
    {
      // 直接发送夹爪命令到话题
      auto publisher = getNode()->create_publisher<je_software::msg::EndEffectorCommandLR>(
        cfg.topic, rclcpp::QoS(10).reliable());

      je_software::msg::EndEffectorCommandLR cmd_lr;
      je_software::msg::EndEffectorCommand cmd;
      cmd.mode = cfg.mode;
      cmd.position = cfg.position;
      cmd.preset = cfg.preset;
      cmd.command = cfg.command;
      cmd.torque = cfg.torque;

      cmd_lr.left_valid = cfg.leftValid;
      cmd_lr.right_valid = cfg.rightValid;
      cmd_lr.left = cmd;
      cmd_lr.right = cmd;

      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] Publishing gripper command: mode=%d, position=%.2f, preset=%d, command=%s, torque=%.2f, left=%d, right=%d to '%s'",
        log_utils::bjtNowString().c_str(),
        cfg.mode,
        cfg.position,
        cfg.preset,
        cfg.command.c_str(),
        cfg.torque,
        cfg.leftValid,
        cfg.rightValid,
        cfg.topic.c_str());

      publisher->publish(cmd_lr);

      // 等待一段时间让命令执行
      std::this_thread::sleep_for(std::chrono::seconds(1));

      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] Gripper command executed successfully",
        log_utils::bjtNowString().c_str());

      // 发送成功事件
      this->postEvent(new smacc2::EvCbSuccess<cl_moveit2z::CbCtrlGripper, OrGripper>());
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        log_utils::bizLogger(),
        "[%s] Failed to execute gripper command: %s",
        log_utils::bjtNowString().c_str(),
        e.what());

      // 发送失败事件
      this->postEvent(new smacc2::EvCbFailure<cl_moveit2z::CbCtrlGripper, OrGripper>());
    }
  }

  void onExit()
  {
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] EXIT TEST_GRIPPER",
      log_utils::bjtNowString().c_str());
  }
};

}  // namespace je_arm_pcb_inspection_sm


