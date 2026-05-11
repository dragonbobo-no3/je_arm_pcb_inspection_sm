#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StWaitResources;

namespace work_substates
{

struct StInspect;

namespace inspect_substates
{

struct StInspectWaitPlaceResult : smacc2::SmaccState<StInspectWaitPlaceResult, StInspect>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvResumeToWaitResources, StWaitResources>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kInspectResumeSubstateId),
      std::string(sm_data::kInspectSubstateWaitPlaceResult));
    this->setGlobalSMData(
      std::string(sm_data::kPlaceResumeSubstateId),
      std::string(sm_data::kPlaceSubstateLPregrasp));
    this->setGlobalSMData(std::string(sm_data::kPlaceResumeFromPause), false);
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWaitResourcesState));
    RCLCPP_INFO(
      getLogger(),
      "WORK::INSPECT::WAIT_PLACE_RESULT - both arms parked at dual_pick_p2, entering WAIT_RESOURCES until place target is ready");
    this->template postEvent<EvResumeToWaitResources>();
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm