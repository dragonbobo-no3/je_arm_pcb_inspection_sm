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
struct StIdle;
struct StWork;
struct StPause;

/// WAIT_RESOURCES 状态：等待资源就绪（PCB、放置槽位等）
struct StWaitResources : smacc2::SmaccState<StWaitResources, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvCanWork, StWork>,
    smacc2::Transition<EvWaitTimeout, StIdle>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void runtimeConfigure() {}

  void onEntry() 
  { 
    this->requiresComponent(flow_);
    enteredTime_ = std::chrono::steady_clock::now();
    transitionPosted_ = false;
    flow_->setResumeTarget(sm_data::kWaitResourcesState);
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER WAIT_RESOURCES",
      log_utils::bjtNowString().c_str());
  }

  void update()
  {
    if (transitionPosted_)
    {
      return;
    }

    const bool canWork = flow_->isWorkReady();
    if (canWork)
    {
      transitionPosted_ = true;
      RCLCPP_INFO(log_utils::bizLogger(), "[%s] TRANSITION WAIT_RESOURCES --(EvCanWork)--> WORK", log_utils::bjtNowString().c_str());
      this->template postEvent<EvCanWork>();
      return;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - enteredTime_);
    if (elapsed.count() >= flow_->waitTimeoutSeconds())
    {
      transitionPosted_ = true;
      RCLCPP_WARN(log_utils::bizLogger(), "[%s] TRANSITION WAIT_RESOURCES --(EvWaitTimeout)--> IDLE", log_utils::bjtNowString().c_str());
      this->template postEvent<EvWaitTimeout>();
    }
  }

  void onExit() 
  { 
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT WAIT_RESOURCES", log_utils::bjtNowString().c_str());
  }

private:
  CpTopLevelFlow * flow_;
  std::chrono::steady_clock::time_point enteredTime_;
  bool transitionPosted_{false};
};

}  // namespace je_arm_pcb_inspection_sm
