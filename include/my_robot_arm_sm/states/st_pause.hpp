#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "my_robot_arm_sm/sm_data.hpp"

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

// 前向声明
struct StBack;
struct StPauseResumeRouter;

/// PAUSE 状态：人工恢复
/// 故障、可恢复异常的集中处理，等待操作员手动指令
struct StPause : smacc2::SmaccState<StPause, SmMyRobotArm>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvFaultToBack, StBack>,
    smacc2::Transition<EvKeyBack, StBack>,
    smacc2::Transition<EvKeyResume, StPauseResumeRouter>
  > reactions;

  void onEntry() 
  { 
    std::string resumeState = sm_data::kWaitResourcesState;
    this->getGlobalSMData(std::string(sm_data::kResumeStateId), resumeState);

    RCLCPP_WARN(
      getLogger(),
      "StPause::onEntry - waiting manual intervention (debug keys handled by keyboard mapper), resume target=%s",
      resumeState.c_str()); 
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StPause::onExit"); 
  }
};

}  // namespace my_robot_arm_sm
