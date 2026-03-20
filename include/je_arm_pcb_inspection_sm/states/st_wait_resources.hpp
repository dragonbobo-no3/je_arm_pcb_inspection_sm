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
struct StBackToIdle;
struct StWork;
struct StDelay;
struct StPause;

/// WAIT_RESOURCES 状态：等待资源就绪（PCB、放置槽位等）
struct StWaitResources : smacc2::SmaccState<StWaitResources, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvCanWork, StWork>,
    smacc2::Transition<EvWaitTimeout, StBackToIdle>,
    smacc2::Transition<EvBackToIdleRequested, StBackToIdle>,
    smacc2::Transition<EvDelayRequested, StDelay>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

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

    auto node = this->getStateMachine().getNode();
    checkTimer_ = node->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]()
      {
        if (transitionPosted_)
        {
          return;
        }

        const bool canWork = flow_->isWorkReady();
        if (canWork)
        {
          transitionPosted_ = true;
          if (checkTimer_)
          {
            checkTimer_->cancel();
          }
          RCLCPP_INFO(
            log_utils::bizLogger(),
            "[%s] TRANSITION WAIT_RESOURCES --(EvCanWork)--> WORK",
            log_utils::bjtNowString().c_str());
          this->template postEvent<EvCanWork>();
          return;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - enteredTime_);
        if (elapsed.count() >= flow_->waitTimeoutSeconds())
        {
          transitionPosted_ = true;
          if (checkTimer_)
          {
            checkTimer_->cancel();
          }
          RCLCPP_WARN(
            log_utils::bizLogger(),
            "[%s] TRANSITION WAIT_RESOURCES --(EvWaitTimeout)--> BACK_TO_IDLE",
            log_utils::bjtNowString().c_str());
          this->template postEvent<EvWaitTimeout>();
        }
      });
  }

  void onExit() 
  { 
    if (checkTimer_)
    {
      checkTimer_->cancel();
      checkTimer_.reset();
    }
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT WAIT_RESOURCES", log_utils::bjtNowString().c_str());
  }

private:
  CpTopLevelFlow * flow_;
  rclcpp::TimerBase::SharedPtr checkTimer_;
  std::chrono::steady_clock::time_point enteredTime_;
  bool transitionPosted_{false};
};

}  // namespace je_arm_pcb_inspection_sm
