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

namespace place_substates
{
struct StPlaceResumeRouter;
}  // namespace place_substates

struct StPlace : smacc2::SmaccState<StPlace, StWork, place_substates::StPlaceResumeRouter>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->requiresComponent(flow_);
    bool placeResumeFromPause = false;
    this->getGlobalSMData(std::string(sm_data::kPlaceResumeFromPause), placeResumeFromPause);

    if (!placeResumeFromPause)
    {
      this->setGlobalSMData(
        std::string(sm_data::kPlaceResumeSubstateId),
        std::string(sm_data::kPlaceSubstateLPregrasp));
    }

    this->setGlobalSMData(std::string(sm_data::kPlaceResumeFromPause), false);
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    RCLCPP_INFO(
      getLogger(),
      "WORK::PLACE onEntry - nested flow started (L_PREGRASP->CARTESIAN_DOWN->GRIPPER_OPEN->CARTESIAN_UP->GRIPPER_CLOSE)");
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

#include "je_arm_pcb_inspection_sm/states/work_substates/place_substates/st_place_resume_router.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/place_substates/st_place_pre.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/place_substates/st_place_move.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/place_substates/st_place_release.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/place_substates/st_place_retreat.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/place_substates/st_place_gripper_close.hpp"
