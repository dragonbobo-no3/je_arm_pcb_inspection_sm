#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_end_effector.hpp>
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

class CbMovePcbTargetPose : public cl_moveit2z::CbMoveEndEffector
{
public:
  CbMovePcbTargetPose()
  {
    tip_link_ = "Link7";
  }

  void onEntry() override
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.orientation.w = 1.0;

    auto * stateMachine = this->getStateMachine();
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetFrameId), pose.header.frame_id);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetX), pose.pose.position.x);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetY), pose.pose.position.y);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetZ), pose.pose.position.z);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQx), pose.pose.orientation.x);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQy), pose.pose.orientation.y);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQz), pose.pose.orientation.z);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQw), pose.pose.orientation.w);

    targetPose = pose;

    RCLCPP_INFO(
      getLogger(),
      "[CbMovePcbTargetPose] Simulated PCBA pose -> frame=%s pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
      targetPose.header.frame_id.c_str(),
      targetPose.pose.position.x,
      targetPose.pose.position.y,
      targetPose.pose.position.z,
      targetPose.pose.orientation.x,
      targetPose.pose.orientation.y,
      targetPose.pose.orientation.z,
      targetPose.pose.orientation.w);

    cl_moveit2z::CbMoveEndEffector::onEntry();
  }
};

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
    smacc2::Transition<smacc2::EvCbSuccess<CbMovePcbTargetPose, OrArm>, StGripperOpen>,
    smacc2::Transition<smacc2::EvCbFailure<CbMovePcbTargetPose, OrArm>, StPause>,
    smacc2::Transition<EvAtPregrasp, StGripperOpen>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrArm, CbMovePcbTargetPose>();
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLPregraspP3));
    RCLCPP_INFO(
      getLogger(),
      "WORK::PICK::L_PREGRASP::P3 - moving to simulated PCBA end-effector pose  [bypass: 'n']");
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
