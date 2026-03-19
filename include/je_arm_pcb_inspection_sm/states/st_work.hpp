#pragma once

#include <chrono>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;

// 前向声明
struct StBack;
struct StDelay;
struct StPause;

namespace work_substates
{
struct StWorkResumeRouter;
}  // namespace work_substates

/// WORK 状态：执行主要流程（拿起 -> 检查 -> 选择箱 -> 放置）
struct StWork : smacc2::SmaccState<StWork, SmJeArmPcbInspection, work_substates::StWorkResumeRouter>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvResourcesUnavailable, StBack>,
    smacc2::Transition<EvDelayRequested, StDelay>,
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
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER WORK (PICK->INSPECT->SELECT_BIN->PLACE)",
      log_utils::bjtNowString().c_str());

    auto node = this->getStateMachine().getNode();
    syncTimer_ = node->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]()
      {
        // Keep flow-backed blackboard variables fresh (e.g. PCB detection),
        // but do not trigger transitions from this state-level sync loop.
        (void)flow_->isWorkReady();
      });
  }

  

  void onExit() 
  { 
    if (syncTimer_)
    {
      syncTimer_->cancel();
      syncTimer_.reset();
    }
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT WORK", log_utils::bjtNowString().c_str());
  }

private:
  CpTopLevelFlow * flow_;
  rclcpp::TimerBase::SharedPtr syncTimer_;
};

}  // namespace je_arm_pcb_inspection_sm

#include "je_arm_pcb_inspection_sm/states/work_substates/st_work_resume_router.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/st_pick.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/st_inspect.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/st_select_bin.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/st_place.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/st_place_decision.hpp"
