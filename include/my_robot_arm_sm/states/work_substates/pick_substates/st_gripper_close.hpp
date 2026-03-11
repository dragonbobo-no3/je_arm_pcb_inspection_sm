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

struct StCartesianUp;

struct StGripperClose : smacc2::SmaccState<StGripperClose, StPick>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvGripperClosed, StCartesianUp>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateGripperClose));
    RCLCPP_INFO(getLogger(), "WORK::PICK::GRIPPER_CLOSE - press 'n' => EvGripperClosed");
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace my_robot_arm_sm
