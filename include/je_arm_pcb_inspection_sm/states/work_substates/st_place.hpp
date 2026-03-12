#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StWork;
struct StPause;

namespace work_substates
{

struct StPlaceDecision;

struct StPlace : smacc2::SmaccState<StPlace, StWork>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPlaceDone, StPlaceDecision>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->requiresComponent(flow_);
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    RCLCPP_INFO(getLogger(), "WORK::PLACE onEntry - press 'n' to consume PCB and post EvPlaceDone");
  }

  void onExit()
  {
    // 消耗当前待处理PCB，避免在资源未更新前无限循环
    flow_->setPcbPresent(false);
  }

private:
  CpTopLevelFlow * flow_;
};

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
