#pragma once

#include <chrono>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "my_robot_arm_sm/components/cp_top_level_flow.hpp"
#include "my_robot_arm_sm/sm_data.hpp"

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

// 前向声明
struct StBack;
struct StPause;

/// WORK 状态：执行主要流程（拿起 -> 检查 -> 选择箱 -> 放置）
struct StWork : smacc2::SmaccState<StWork, SmMyRobotArm>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvCanWork, StWork>,
    smacc2::Transition<EvResourcesUnavailable, StBack>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void runtimeConfigure() {}

  void onEntry() 
  { 
    this->requiresComponent(flow_);
    enteredTime_ = std::chrono::steady_clock::now();
    transitionPosted_ = false;
    flow_->setResumeTarget(sm_data::kWorkState);
    RCLCPP_WARN(
      getLogger(),
      "StWork::onEntry - top-level work loop (debug keys handled by keyboard mapper)"); 
  }

  void update()
  {
    if (transitionPosted_)
    {
      return;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - enteredTime_);
    if (elapsed.count() < flow_->workCycleSeconds())
    {
      return;
    }

    transitionPosted_ = true;
    if (flow_->isWorkReady())
    {
      RCLCPP_INFO(getLogger(), "Simulated work cycle done -> posting EvCanWork");
      this->template postEvent<EvCanWork>();
    }
    else
    {
      RCLCPP_WARN(getLogger(), "Resources unavailable -> posting EvResourcesUnavailable");
      this->template postEvent<EvResourcesUnavailable>();
    }
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StWork::onExit"); 
  }

private:
  CpTopLevelFlow * flow_;
  std::chrono::steady_clock::time_point enteredTime_;
  bool transitionPosted_{false};
};

}  // namespace my_robot_arm_sm
