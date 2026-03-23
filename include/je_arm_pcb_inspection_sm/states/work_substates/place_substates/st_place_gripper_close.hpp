#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_ctrl_gripper.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/gripper_command_loader.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StBack;

namespace work_substates
{

struct StPlace;

namespace place_substates
{

struct StPlaceGripperClose : smacc2::SmaccState<StPlaceGripperClose, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbCtrlGripper, OrArm>, StBack>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbCtrlGripper, OrArm>, StPause>,
    smacc2::Transition<EvPlaceDone, StBack>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    const auto cfg = je_arm_pcb_inspection_sm::utils::loadGripperCommandConfig("place_close");
    configure_orthogonal<OrArm, cl_moveit2z::CbCtrlGripper>(
      cfg.mode,
      cfg.position,
      cfg.preset,
      cfg.leftValid,
      cfg.rightValid,
      cfg.topic);
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPlaceResumeSubstateId),
      std::string(sm_data::kPlaceSubstateGripperClose));
    RCLCPP_INFO(getLogger(), "WORK::PLACE::GRIPPER_CLOSE - executing CbCtrlGripper POSITION=0.0 [bypass: 'n']");
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
