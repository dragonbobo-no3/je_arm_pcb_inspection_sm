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
struct StWaitResources;
struct StPause;

/// BACK 状态：回零位置
struct StBack : smacc2::SmaccState<StBack, SmMyRobotArm>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvBackDone, StWaitResources>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void runtimeConfigure() {}

  void onEntry() 
  { 
    this->requiresComponent(flow_);
    enteredTime_ = std::chrono::steady_clock::now();
    transitionPosted_ = false;
    flow_->setResumeTarget(sm_data::kBackState);
    RCLCPP_WARN(getLogger(), "StBack::onEntry - homing arm (debug keys handled by keyboard mapper)"); 
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
      RCLCPP_INFO(getLogger(), "Back motion complete -> posting EvBackDone");
      this->template postEvent<EvBackDone>();
    }
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StBack::onExit"); 
  }

private:
  CpTopLevelFlow * flow_;
  std::chrono::steady_clock::time_point enteredTime_;
  bool transitionPosted_{false};
};

}  // namespace my_robot_arm_sm
