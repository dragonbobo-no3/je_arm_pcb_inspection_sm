#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

// 前向声明
struct StWaitResources;
struct StPause;

/// BACK 状态：回零位置
struct StBack : smacc2::SmaccState<StBack, SmMyRobotArm>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvBackDone, StWaitResources>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry() 
  { 
    RCLCPP_WARN(getLogger(), "StBack::onEntry"); 
    // TODO: 执行回零位置动作
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StBack::onExit"); 
  }
};

}  // namespace my_robot_arm_sm
