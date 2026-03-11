#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "my_robot_arm_sm/components/cp_top_level_flow.hpp"
#include "my_robot_arm_sm/sm_data.hpp"

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

// 前向声明
struct StBack;
struct StPause;

namespace work_substates
{
struct StWorkResumeRouter;
}  // namespace work_substates

/// WORK 状态：执行主要流程（拿起 -> 检查 -> 选择箱 -> 放置）
struct StWork : smacc2::SmaccState<StWork, SmMyRobotArm, work_substates::StWorkResumeRouter>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvResourcesUnavailable, StBack>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void runtimeConfigure() {}

  void onEntry() 
  { 
    this->requiresComponent(flow_);
    bool resumeFromPause = false;
    this->getGlobalSMData(std::string(sm_data::kResumeFromPause), resumeFromPause);

    if (!resumeFromPause)
    {
      this->setGlobalSMData(
        std::string(sm_data::kWorkResumeSubstateId),
        std::string(sm_data::kWorkSubstatePick));
    }
    flow_->setResumeTarget(sm_data::kWorkState);
    RCLCPP_WARN(
      getLogger(),
      "StWork::onEntry - hierarchical WORK flow started (PICK->INSPECT->SELECT_BIN->PLACE)"); 
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StWork::onExit"); 
  }

private:
  CpTopLevelFlow * flow_;
};

}  // namespace my_robot_arm_sm

#include "my_robot_arm_sm/states/work_substates/st_work_resume_router.hpp"
#include "my_robot_arm_sm/states/work_substates/st_pick.hpp"
#include "my_robot_arm_sm/states/work_substates/st_inspect.hpp"
#include "my_robot_arm_sm/states/work_substates/st_select_bin.hpp"
#include "my_robot_arm_sm/states/work_substates/st_place.hpp"
#include "my_robot_arm_sm/states/work_substates/st_place_decision.hpp"
