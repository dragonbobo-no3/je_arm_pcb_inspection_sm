#pragma once

#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <cl_moveit2z/client_behaviors/cb_move_joints.hpp>

namespace je_arm_pcb_inspection_sm
{
namespace work_substates
{
namespace inspect_substates
{

class CbMoveInspectHandoverJointState : public cl_moveit2z::CbMoveJoints
{
public:
  explicit CbMoveInspectHandoverJointState(const std::string & stateKey)
  : stateKey_(stateKey)
  {}

  void onEntry() override
  {
    loadJointStateFromYaml();
    cl_moveit2z::CbMoveJoints::onEntry();
  }

private:
  void loadJointStateFromYaml()
  {
    const std::string packageShareDir =
      ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm");
    const std::string sourcePath =
      packageShareDir + "/config/move_group_client/joint_states/inspect_handover_states.yaml";

    if (!std::filesystem::exists(sourcePath))
    {
      throw std::runtime_error("inspect handover yaml not found: " + sourcePath);
    }

    const YAML::Node root = YAML::LoadFile(sourcePath);
    const YAML::Node stateNode = root[stateKey_];
    if (!stateNode)
    {
      throw std::runtime_error("inspect handover state key not found in yaml: " + stateKey_);
    }

    const YAML::Node jointStatesNode = stateNode["joint_states"];
    if (!jointStatesNode)
    {
      throw std::runtime_error("inspect handover joint_states missing for key: " + stateKey_);
    }

    jointValueTarget_.clear();
    for (YAML::const_iterator it = jointStatesNode.begin(); it != jointStatesNode.end(); ++it)
    {
      jointValueTarget_[it->first.as<std::string>()] = it->second.as<double>();
    }

    if (stateNode["scaling_factor"])
    {
      scalingFactor_ = stateNode["scaling_factor"].as<double>();
    }
    else
    {
      scalingFactor_ = std::nullopt;
    }

    RCLCPP_INFO(
      getLogger(),
      "[CbMoveInspectHandoverJointState] key=%s loaded %zu joints from %s",
      stateKey_.c_str(),
      jointValueTarget_.size(),
      sourcePath.c_str());
  }

  std::string stateKey_;
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm