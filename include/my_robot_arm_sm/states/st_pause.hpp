#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

// 前向声明
struct StWaitResources;
struct StWork;
struct StBack;

/// PAUSE 状态：人工恢复
/// 故障、可恢复异常的集中处理，等待操作员手动指令
struct StPause : smacc2::SmaccState<StPause, SmMyRobotArm>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvKeyResume, StWaitResources>,
    smacc2::Transition<EvKeyResume, StWork>,
    smacc2::Transition<EvFaultToBack, StBack>,
    smacc2::Transition<EvKeyBack, StBack>
  > reactions;

  void onEntry() 
  { 
    RCLCPP_WARN(getLogger(), "StPause::onEntry - Waiting for manual intervention"); 
    // TODO: 高亮LED/蜂鸣器，等待操作员指令
    // TODO: 根据黑板变量 resume_state_id 决定 EvKeyResume 转移到哪个状态
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StPause::onExit"); 
  }
};

}  // namespace my_robot_arm_sm
