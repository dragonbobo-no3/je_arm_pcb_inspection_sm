#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_known_state.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;

namespace work_substates
{

struct StInspect;

namespace inspect_substates
{

struct StInspectWaitPlaceResult;

struct StInspectReturnWaitPose : smacc2::SmaccState<StInspectReturnWaitPose, StInspect>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<cl_moveit2z::CbMoveKnownState, OrBothArms>,
      StInspectWaitPlaceResult>,
    smacc2::Transition<
      smacc2::EvCbFailure<cl_moveit2z::CbMoveKnownState, OrBothArms>,
      StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrBothArms, cl_moveit2z::CbMoveKnownState>(
      "je_arm_pcb_inspection_sm",
      "config/move_group_client/joint_states/dual_pick_p2.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kInspectResumeSubstateId),
      std::string(sm_data::kInspectSubstateReturnWaitPose));
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWorkState));
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstateInspect));
    RCLCPP_INFO(
      getLogger(),
      "WORK::INSPECT::RETURN_WAIT_POSE - moving both arms back to dual_pick_p2 before waiting for place result");
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm