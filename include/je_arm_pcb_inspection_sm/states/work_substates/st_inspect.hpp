#pragma once

#include <map>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_joints.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StWork;
struct StPause;
struct StDelay;

namespace work_substates
{

struct StSelectBin;

struct StInspect : smacc2::SmaccState<StInspect, StWork>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbMoveJoints, OrArm>, StDelay>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbMoveJoints, OrArm>, StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal_runtime<OrArm, cl_moveit2z::CbMoveJoints>(
      [](cl_moveit2z::CbMoveJoints & bh, StInspect & state)
      {
        bh.jointValueTarget_ = {
          {"joint1", 0.117},
          {"joint2", -1.580},
          {"joint3", 1.622},
          {"joint4", 1.422},
          {"joint5", -0.244},
          {"joint6", 0.274},
          {"joint7", 0.287}
        };

        RCLCPP_INFO(
          state.getLogger(),
          "WORK::INSPECT - runtime configured joint target: [0.117, -1.580, 1.622, 1.422, -0.244, 0.274, 0.287]");
      });
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstateInspect));
    this->setGlobalSMData(
      std::string(sm_data::kWorkDelayNextSubstateId),
      std::string(sm_data::kWorkSubstateSelectBin));
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId),
      std::string(sm_data::kWorkState));
    RCLCPP_INFO(getLogger(), "WORK::INSPECT onEntry - executing joint move to inspection pose");
  }
};

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
