#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

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

  void onEntry() 
  { 
    RCLCPP_WARN(getLogger(), "StWaitResources::onEntry"); 
    // TODO: 启动资源监听线程/定时器
  }

  void onExit() 
  { 
    RCLCPP_WARN(getLogger(), "StWaitResources::onExit"); 
  }
};

}  // namespace my_robot_arm_sm
