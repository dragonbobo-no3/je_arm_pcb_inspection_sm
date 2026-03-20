#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;

namespace work_substates
{

struct StPlace;

namespace place_substates
{

struct StPlaceCartesianUp;

struct StPlaceGripperOpen : smacc2::SmaccState<StPlaceGripperOpen, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPlaceReleased, StPlaceCartesianUp>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPlaceResumeSubstateId),
      std::string(sm_data::kPlaceSubstateGripperOpen));
    RCLCPP_INFO(getLogger(), "WORK::PLACE::GRIPPER_OPEN - press 'n' to release");
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
