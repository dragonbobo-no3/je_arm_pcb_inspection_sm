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

namespace work_substates
{

struct StPlace;

namespace place_substates
{

struct StPlaceGripperClose;

struct StPlaceCartesianUp : smacc2::SmaccState<StPlaceCartesianUp, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<cl_moveit2z::CbMoveEndEffectorLinearSeeded, OrArm>,
      StPlaceGripperClose>,
    smacc2::Transition<
      smacc2::EvCbFailure<cl_moveit2z::CbMoveEndEffectorLinearSeeded, OrArm>,
      StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal_runtime<OrArm, cl_moveit2z::CbMoveEndEffectorLinearSeeded>(
      [](cl_moveit2z::CbMoveEndEffectorLinearSeeded & bh, StPlaceCartesianUp & state)
      {
        bh.tip_link_ = "Link7";
        bh.linearStepMeters_ = 0.005;
        bh.planningTimeSec_ = 1.0;
        bh.targetPose = computePlacePose(state, 0.10);
      });
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPlaceResumeSubstateId),
      std::string(sm_data::kPlaceSubstateCartesianUp));
    RCLCPP_INFO(getLogger(), "WORK::PLACE::CARTESIAN_UP - moving to place pose with x + 0.10m");
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
