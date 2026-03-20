#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{
namespace work_substates
{
namespace place_substates
{

template <typename TState>
geometry_msgs::msg::PoseStamped computePlacePose(TState & state, double xOffset)
{
  auto * stateMachine = &state.getStateMachine();

  std::string frameId = "base_link";
  double baseX = -0.4494;
  double baseY = 0.3586;
  double baseZ = -0.0047;
  double baseQx = 0.0048;
  double baseQy = -0.7067;
  double baseQz = -0.0125;
  double baseQw = 0.7074;

  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetFrameId), frameId);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetX), baseX);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetY), baseY);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetZ), baseZ);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQx), baseQx);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQy), baseQy);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQz), baseQz);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQw), baseQw);


  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frameId;
  pose.pose.position.x = baseX + xOffset;
  pose.pose.position.y = baseY;
  pose.pose.position.z = baseZ;
  pose.pose.orientation.x = baseQx;
  pose.pose.orientation.y = baseQy;
  pose.pose.orientation.z = baseQz;
  pose.pose.orientation.w = baseQw;

  return pose;
}

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
