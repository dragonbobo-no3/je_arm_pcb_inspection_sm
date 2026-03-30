#pragma once

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace je_arm_pcb_inspection_sm
{
namespace utils
{

inline bool loadWaitResourcesOffset(
  double & offsetX, double & offsetY, double & offsetZ, std::string & sourcePath)
{
  try
  {
    sourcePath =
      ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm") +
      "/config/move_group_client/cartesian_offsets/wait_resources_offset.yaml";

    const YAML::Node root = YAML::LoadFile(sourcePath);
    const YAML::Node offsetNode = root["offset"] ? root["offset"] : root;

    offsetX = offsetNode["x"] ? offsetNode["x"].as<double>() : 0.0;
    offsetY = offsetNode["y"] ? offsetNode["y"].as<double>() : 0.0;
    offsetZ = offsetNode["z"] ? offsetNode["z"].as<double>() : 0.0;
    return true;
  }
  catch (...)
  {
    sourcePath.clear();
    offsetX = 0.0;
    offsetY = 0.0;
    offsetZ = 0.0;
    return false;
  }
}

}  // namespace utils
}  // namespace je_arm_pcb_inspection_sm