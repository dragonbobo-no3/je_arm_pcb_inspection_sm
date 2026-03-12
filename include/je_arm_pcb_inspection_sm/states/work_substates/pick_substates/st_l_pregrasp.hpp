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

struct StPick;

namespace pick_substates
{

struct StGripperOpen;

struct StLPregrasp : smacc2::SmaccState<StLPregrasp, StPick>
{
  using SmaccState::SmaccState;

  // EvCbSuccess fires automatically when CbMoveKnownState completes
  // EvAtPregrasp is kept as a manual bypass key ('n') for testing/debugging
  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbMoveKnownState, OrArm>, StGripperOpen>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbMoveKnownState, OrArm>, StPause>,
    smacc2::Transition<EvAtPregrasp, StGripperOpen>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  // Register CbMoveKnownState on OrArm: loads pick pregrasp joint config from yaml
  static void staticConfigure()
  {
    configure_orthogonal<OrArm, cl_moveit2z::CbMoveKnownState>(
      "je_arm_pcb_inspection_sm",
      "config/move_group_client/joint_states/pick.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLPregrasp));
    RCLCPP_INFO(
      getLogger(),
      "WORK::PICK::L_PREGRASP - executing joint move to pregrasp pose  [bypass: 'n']");
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
