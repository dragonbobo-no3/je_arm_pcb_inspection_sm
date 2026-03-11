#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "my_robot_arm_sm/events.hpp"
#include "my_robot_arm_sm/sm_data.hpp"

namespace my_robot_arm_sm
{

struct StPause;

namespace work_substates
{

struct StPick;

namespace pick_substates
{

struct StGripperOpen;

struct StLPregrasp : smacc2::SmaccState<StLPregrasp, StPick>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvAtPregrasp, StGripperOpen>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLPregrasp));
    RCLCPP_INFO(getLogger(), "WORK::PICK::L_PREGRASP - press 'n' => EvAtPregrasp");
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace my_robot_arm_sm
