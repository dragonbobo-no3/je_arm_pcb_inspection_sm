#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_end_effector_linear_seeded.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/place_substates/place_pose_utils.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StDelay;

namespace work_substates
{

struct StPlace;

namespace place_substates
{

struct StPlaceGripperOpen;

struct StPlaceCartesianDown : smacc2::SmaccState<StPlaceCartesianDown, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<cl_moveit2z::CbMoveEndEffectorLinearSeeded, OrArm>,
      StDelay>,
    smacc2::Transition<
      smacc2::EvCbFailure<cl_moveit2z::CbMoveEndEffectorLinearSeeded, OrArm>,
      StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal_runtime<OrArm, cl_moveit2z::CbMoveEndEffectorLinearSeeded>(
      [](cl_moveit2z::CbMoveEndEffectorLinearSeeded & bh, StPlaceCartesianDown & state)
      {
        bh.tip_link_ = "Link7";
        bh.targetPose = computePlacePose(state, 0.0);
      });
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPlaceResumeSubstateId),
      std::string(sm_data::kPlaceSubstateCartesianDown));
    this->setGlobalSMData(
      std::string(sm_data::kPlaceDelayNextSubstateId),
      std::string(sm_data::kPlaceSubstateGripperOpen));
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId),
      std::string(sm_data::kWorkState));
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    this->setGlobalSMData(
      std::string(sm_data::kWorkDelayNextSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    RCLCPP_INFO(getLogger(), "WORK::PLACE::CARTESIAN_DOWN - moving to place pose");
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
