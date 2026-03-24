#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;

// 前向声明
struct StActivate;
struct StWaitResources;
struct StDelay;
struct StPause;
struct StTestGripper;

namespace work_substates
{
struct StPick;
}

/// IDLE 状态：系统空闲，等待启动指令
struct StIdle : smacc2::SmaccState<StIdle, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvStartWork, StActivate>,
    smacc2::Transition<EvTestGripper, StTestGripper>,
    smacc2::Transition<EvDelayRequested, StDelay>
  > reactions;

  void onEntry() 
  { 
    RCLCPP_INFO(log_utils::bizLogger(), "[%s] ENTER IDLE", log_utils::bjtNowString().c_str());
  }

  void onExit() 
  { 
    // Initialize activate resume pointer when leaving Idle to start work.
    this->setGlobalSMData(
      std::string(sm_data::kActivateResumeSubstateId),
      std::string(sm_data::kActivateSubstateP1));
    this->setGlobalSMData(
      std::string(sm_data::kActivateDelayNextSubstateId),
      std::string(sm_data::kActivateSubstateP1));

    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT IDLE", log_utils::bjtNowString().c_str());
  }
};

}  // namespace je_arm_pcb_inspection_sm
