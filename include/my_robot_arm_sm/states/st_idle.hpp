#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

// 前向声明
struct StWaitResources;
struct StPause;

namespace work_substates
{
struct StPick;
}

/// IDLE 状态：系统空闲，等待启动指令
struct StIdle : smacc2::SmaccState<StIdle, SmMyRobotArm>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvStartWork, StWaitResources>
  > reactions;

  void onEntry() 
  { 
    RCLCPP_WARN(getLogger(), "StIdle::onEntry - debug key: s => EvStartWork"); 
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StIdle::onExit"); 
  }
};

}  // namespace my_robot_arm_sm
