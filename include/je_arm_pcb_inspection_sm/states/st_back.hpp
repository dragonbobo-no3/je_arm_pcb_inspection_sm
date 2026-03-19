#pragma once

#include <chrono>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;

// 前向声明
struct StWaitResources;
struct StDelay;
struct StPause;

/// BACK 状态：回零位置
struct StBack : smacc2::SmaccState<StBack, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvBackDone, StWaitResources>,
    smacc2::Transition<EvDelayRequested, StDelay>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void runtimeConfigure() {}

  void onEntry() 
  { 
    this->requiresComponent(flow_);
    enteredTime_ = std::chrono::steady_clock::now();
    transitionPosted_ = false;
    flow_->setResumeTarget(sm_data::kBackState);
    RCLCPP_INFO(log_utils::bizLogger(), "[%s] ENTER BACK (homing)", log_utils::bjtNowString().c_str());
  }

  void update()
  {
    if (transitionPosted_)
    {
      return;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - enteredTime_);
    if (elapsed.count() >= flow_->backHomeSeconds())
    {
      transitionPosted_ = true;
      RCLCPP_INFO(log_utils::bizLogger(), "[%s] TRANSITION BACK --(EvBackDone)--> WAIT_RESOURCES", log_utils::bjtNowString().c_str());
      this->template postEvent<EvBackDone>();
    }
  }

  void onExit() 
  { 
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT BACK", log_utils::bjtNowString().c_str());
  }

private:
  CpTopLevelFlow * flow_;
  std::chrono::steady_clock::time_point enteredTime_;
  bool transitionPosted_{false};
};

}  // namespace je_arm_pcb_inspection_sm
