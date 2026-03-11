#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "my_robot_arm_sm/sm_data.hpp"

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

struct StWaitResources;
struct StWork;
struct StBack;

/// PAUSE 恢复路由状态：根据黑板中的 resume_state_id 恢复到目标状态
struct StPauseResumeRouter : smacc2::SmaccState<StPauseResumeRouter, SmMyRobotArm>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvResumeToWaitResources, StWaitResources>,
    smacc2::Transition<EvResumeToWork, StWork>,
    smacc2::Transition<EvResumeToBack, StBack>
  > reactions;

  void onEntry()
  {
    std::string resumeState = sm_data::kWaitResourcesState;
    this->getGlobalSMData(std::string(sm_data::kResumeStateId), resumeState);
    this->setGlobalSMData(std::string(sm_data::kResumeFromPause), false);

    RCLCPP_INFO(getLogger(), "StPauseResumeRouter::onEntry - resume target: %s", resumeState.c_str());

    if (resumeState == sm_data::kWorkState)
    {
      this->setGlobalSMData(std::string(sm_data::kResumeFromPause), true);
      this->template postEvent<EvResumeToWork>();
    }
    else if (resumeState == sm_data::kBackState)
    {
      this->template postEvent<EvResumeToBack>();
    }
    else
    {
      this->template postEvent<EvResumeToWaitResources>();
    }
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StPauseResumeRouter::onExit");
  }
};

}  // namespace my_robot_arm_sm