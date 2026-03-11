#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

// 前向声明
struct StBack;
struct StPause;

/// WORK 状态：执行主要流程（拿起 -> 检查 -> 选择箱 -> 放置）
struct StWork : smacc2::SmaccState<StWork, SmMyRobotArm>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvCanWork, StWork>,
    smacc2::Transition<EvResourcesUnavailable, StBack>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry() 
  { 
    RCLCPP_WARN(getLogger(), "StWork::onEntry"); 
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StWork::onExit"); 
  }
};

}  // namespace my_robot_arm_sm
