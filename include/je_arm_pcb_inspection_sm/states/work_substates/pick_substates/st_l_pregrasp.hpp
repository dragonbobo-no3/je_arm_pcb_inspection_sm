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

// Forward declarations for the three pregrasp waypoint sub-states
struct StLPregraspP1;
struct StLPregraspP2;
struct StLPregraspP3;

// ---- Composite parent: StLPregrasp ----------------------------------------
// Sequences through P1 -> P2 -> P3 before exiting to StGripperOpen.
// EvPauseRequested is NOT listed here; it bubbles up to StPick which handles it.
struct StLPregrasp : smacc2::SmaccState<StLPregrasp, StPick, StLPregraspP1>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "WORK::PICK::L_PREGRASP - composite pregrasp sequence started (P1->P2->P3)");
  }
};

// ---- P1: first approach waypoint ------------------------------------------
struct StLPregraspP1 : smacc2::SmaccState<StLPregraspP1, StLPregrasp>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbMoveKnownState, OrArm>, StLPregraspP2>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbMoveKnownState, OrArm>, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrArm, cl_moveit2z::CbMoveKnownState>(
      "je_arm_pcb_inspection_sm",
      "config/move_group_client/joint_states/pick_p1.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLPregraspP1));
    RCLCPP_INFO(getLogger(), "WORK::PICK::L_PREGRASP::P1 - moving to waypoint 1");
  }
};

// ---- P2: intermediate waypoint --------------------------------------------
struct StLPregraspP2 : smacc2::SmaccState<StLPregraspP2, StLPregrasp>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbMoveKnownState, OrArm>, StLPregraspP3>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbMoveKnownState, OrArm>, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrArm, cl_moveit2z::CbMoveKnownState>(
      "je_arm_pcb_inspection_sm",
      "config/move_group_client/joint_states/pick_p2.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLPregraspP2));
    RCLCPP_INFO(getLogger(), "WORK::PICK::L_PREGRASP::P2 - moving to waypoint 2");
  }
};

// ---- P3: final pregrasp pose ----------------------------------------------
struct StLPregraspP3 : smacc2::SmaccState<StLPregraspP3, StLPregrasp>
{
  using SmaccState::SmaccState;

  // EvAtPregrasp: manual bypass key ('n') for testing/debugging
  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbMoveKnownState, OrArm>, StGripperOpen>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbMoveKnownState, OrArm>, StPause>,
    smacc2::Transition<EvAtPregrasp, StGripperOpen>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrArm, cl_moveit2z::CbMoveKnownState>(
      "je_arm_pcb_inspection_sm",
      "config/move_group_client/joint_states/pick_p3.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLPregraspP3));
    RCLCPP_INFO(
      getLogger(),
      "WORK::PICK::L_PREGRASP::P3 - moving to final pregrasp pose  [bypass: 'n']");
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
