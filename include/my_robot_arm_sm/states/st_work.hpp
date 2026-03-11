#pragma once

#include <chrono>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
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
    enteredTime_ = std::chrono::steady_clock::now();
    transitionPosted_ = false;
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWorkState));
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
    if (elapsed.count() < getCycleSeconds())
    {
      return;
    }

    transitionPosted_ = true;
    if (isWorkReady())
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
  bool getBoolData(const char * key, bool fallback)
  {
    bool value = fallback;
    this->getGlobalSMData(std::string(key), value);
    return value;
  }

  double getCycleSeconds()
  {
    double cycleSec = 2.0;
    this->getGlobalSMData(std::string(sm_data::kWorkCycleSec), cycleSec);
    return cycleSec;
  }

  bool isWorkReady()
  {
    const bool pcbPresent = getBoolData(sm_data::kPcbPresent, true);
    const bool leftSlotFree = getBoolData(sm_data::kLeftSlotFree, true);
    const bool rightSlotFree = getBoolData(sm_data::kRightSlotFree, true);
    return pcbPresent && (leftSlotFree || rightSlotFree);
  }

  std::chrono::steady_clock::time_point enteredTime_;
  bool transitionPosted_{false};
};

}  // namespace my_robot_arm_sm
