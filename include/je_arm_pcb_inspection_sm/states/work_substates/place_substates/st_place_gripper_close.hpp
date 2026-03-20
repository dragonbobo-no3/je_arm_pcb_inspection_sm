#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StBack;

namespace work_substates
{

struct StPlace;

namespace place_substates
{

struct StPlaceGripperClose : smacc2::SmaccState<StPlaceGripperClose, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPlaceDone, StBack>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPlaceResumeSubstateId),
      std::string(sm_data::kPlaceSubstateGripperClose));
    RCLCPP_INFO(getLogger(), "WORK::PLACE::GRIPPER_CLOSE - press 'n' to close and go BACK");
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
