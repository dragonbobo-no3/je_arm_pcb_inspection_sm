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

struct StLRetreat : smacc2::SmaccState<StLRetreat, StPick>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLRetreat));
    RCLCPP_INFO(getLogger(), "WORK::PICK::L_RETREAT - press 'n' => EvPickDone");
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace my_robot_arm_sm
