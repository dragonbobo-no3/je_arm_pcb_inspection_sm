#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StWork;
struct StBack;
struct StPause;

namespace work_substates
{

struct StPlaceDecision : smacc2::SmaccState<StPlaceDecision, StWork>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvCanWork, StWork>,
    smacc2::Transition<EvResourcesUnavailable, StBack>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePlaceDecision));
    RCLCPP_INFO(getLogger(), "WORK::PLACE decision - press 'w' to continue next cycle, or 'u' to back home");
  }
};

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
