#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "my_robot_arm_sm/events.hpp"
#include "my_robot_arm_sm/sm_data.hpp"

namespace my_robot_arm_sm
{

struct StWork;
struct StPause;

namespace work_substates
{

struct StSelectBin;

struct StInspect : smacc2::SmaccState<StInspect, StWork>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvInspectDone, StSelectBin>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstateInspect));
    RCLCPP_INFO(getLogger(), "WORK::INSPECT onEntry - press 'n' to post EvInspectDone");
  }
};

}  // namespace work_substates

}  // namespace my_robot_arm_sm
