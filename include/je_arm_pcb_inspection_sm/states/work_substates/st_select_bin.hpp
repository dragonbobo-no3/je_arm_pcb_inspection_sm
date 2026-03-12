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

struct StPlace;

struct StSelectBin : smacc2::SmaccState<StSelectBin, StWork>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvBinSelected, StPlace>,
    smacc2::Transition<EvResourcesUnavailable, StBack>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstateSelectBin));
    RCLCPP_INFO(getLogger(), "WORK::SELECT_BIN onEntry - press 'n' to post EvBinSelected, or 'u' for EvResourcesUnavailable");
  }
};

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
