#pragma once

#include <chrono>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "my_robot_arm_sm/sm_data.hpp"

namespace my_robot_arm_sm
{

struct SmMyRobotArm;

// 前向声明
struct StIdle;
struct StWork;
struct StPause;

/// WAIT_RESOURCES 状态：等待资源就绪（PCB、放置槽位等）
struct StWaitResources : smacc2::SmaccState<StWaitResources, SmMyRobotArm>
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
    enteredTime_ = std::chrono::steady_clock::now();
    transitionPosted_ = false;
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWaitResourcesState));
    RCLCPP_WARN(
      getLogger(),
      "StWaitResources::onEntry - waiting resources (debug keys handled by keyboard mapper)");
  }

  void update()
  {
    if (transitionPosted_)
    {
      return;
    }

    const bool canWork = isWorkReady();
    if (canWork)
    {
      transitionPosted_ = true;
      RCLCPP_INFO(getLogger(), "Resources ready -> posting EvCanWork");
      this->template postEvent<EvCanWork>();
      return;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - enteredTime_);
    if (elapsed.count() >= getTimeoutSeconds())
    {
      transitionPosted_ = true;
      RCLCPP_WARN(getLogger(), "Wait resources timeout -> posting EvWaitTimeout");
      this->template postEvent<EvWaitTimeout>();
    }
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StWaitResources::onExit"); 
  }

private:
  bool getBoolData(const char * key, bool fallback)
  {
    bool value = fallback;
    this->getGlobalSMData(std::string(key), value);
    return value;
  }

  double getTimeoutSeconds()
  {
    double timeoutSec = 10.0;
    this->getGlobalSMData(std::string(sm_data::kWaitTimeoutSec), timeoutSec);
    return timeoutSec;
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
