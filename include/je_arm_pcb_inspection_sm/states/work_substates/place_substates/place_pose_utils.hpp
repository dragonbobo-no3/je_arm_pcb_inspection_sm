#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/place_offset_loader.hpp"

namespace je_arm_pcb_inspection_sm
{
namespace work_substates
{
namespace place_substates
{

template <typename TState>
geometry_msgs::msg::PoseStamped computePlacePose(TState & state, double xOffset = 0.0)
{
  auto * stateMachine = &state.getStateMachine();

  std::string frameId = "base_link";
  double baseX = -0.3182978287647691;
  double baseY = -0.30670100572292247;
  double baseZ = 0.4037989942810653;
  double baseQx = -0.01659836570155305;
  double baseQy = -0.9941145267183089;
  double baseQz = -0.02580021092886362;
  double baseQw = 0.10389971674502421;

  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetFrameId), frameId);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetX), baseX);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetY), baseY);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetZ), baseZ);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQx), baseQx);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQy), baseQy);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQz), baseQz);
  stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQw), baseQw);

  // Load offset from place_offset.yaml (dynamic, no recompile needed)
  double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
  std::string offsetPath;
  const bool offsetLoaded =
    je_arm_pcb_inspection_sm::utils::loadPlaceOffset(offsetX, offsetY, offsetZ, offsetPath);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frameId;
  pose.pose.position.x = baseX + xOffset + offsetX;
  pose.pose.position.y = baseY + offsetY;
  pose.pose.position.z = baseZ + offsetZ;
  pose.pose.orientation.x = baseQx;
  pose.pose.orientation.y = baseQy;
  pose.pose.orientation.z = baseQz;
  pose.pose.orientation.w = baseQw;

  return pose;
}

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
