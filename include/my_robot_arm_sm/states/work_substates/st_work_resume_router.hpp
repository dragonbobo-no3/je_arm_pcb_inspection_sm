#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "my_robot_arm_sm/events.hpp"
#include "my_robot_arm_sm/sm_data.hpp"
#include "my_robot_arm_sm/states/work_substates/st_pick.hpp"
#include "my_robot_arm_sm/states/work_substates/st_inspect.hpp"
#include "my_robot_arm_sm/states/work_substates/st_select_bin.hpp"
#include "my_robot_arm_sm/states/work_substates/st_place.hpp"
#include "my_robot_arm_sm/states/work_substates/st_place_decision.hpp"

namespace my_robot_arm_sm
{

struct StWork;

namespace work_substates
{

struct StPick;
struct StInspect;
struct StSelectBin;
struct StPlace;
struct StPlaceDecision;

struct StWorkResumeRouter : smacc2::SmaccState<StWorkResumeRouter, StWork>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvWorkResumeToPick, StPick>,
    smacc2::Transition<EvWorkResumeToInspect, StInspect>,
    smacc2::Transition<EvWorkResumeToSelectBin, StSelectBin>,
    smacc2::Transition<EvWorkResumeToPlace, StPlace>,
    smacc2::Transition<EvWorkResumeToPlaceDecision, StPlaceDecision>
  > reactions;

  void onEntry()
  {
    std::string substate = sm_data::kWorkSubstatePick;
    bool resumeFromPause = false;
    this->getGlobalSMData(std::string(sm_data::kResumeFromPause), resumeFromPause);
    this->getGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), substate);

    const bool restorePickSubstate = resumeFromPause && (substate == sm_data::kWorkSubstatePick);
    this->setGlobalSMData(std::string(sm_data::kPickResumeFromPause), restorePickSubstate);
    this->setGlobalSMData(std::string(sm_data::kResumeFromPause), false);

    RCLCPP_INFO(getLogger(), "WORK::RESUME_ROUTER onEntry - target substate: %s", substate.c_str());

    if (substate == sm_data::kWorkSubstateInspect)
    {
      this->template postEvent<EvWorkResumeToInspect>();
    }
    else if (substate == sm_data::kWorkSubstateSelectBin)
    {
      this->template postEvent<EvWorkResumeToSelectBin>();
    }
    else if (substate == sm_data::kWorkSubstatePlace)
    {
      this->template postEvent<EvWorkResumeToPlace>();
    }
    else if (substate == sm_data::kWorkSubstatePlaceDecision)
    {
      this->template postEvent<EvWorkResumeToPlaceDecision>();
    }
    else
    {
      this->template postEvent<EvWorkResumeToPick>();
    }
  }
};

}  // namespace work_substates

}  // namespace my_robot_arm_sm
