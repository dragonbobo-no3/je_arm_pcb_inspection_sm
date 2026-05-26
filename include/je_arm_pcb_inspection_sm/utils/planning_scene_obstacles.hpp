#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm::utils
{

struct ObstacleVector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ObstaclePose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
};

struct ObstacleSpec
{
  std::string id;
  bool enabled = true;
  std::string type = "box";
  std::string frameId;
  ObstaclePose pose;
  ObstacleVector3 size;
  ObstacleVector3 safetyMargin;
};

struct ObstaclesConfig
{
  std::string planningFrame = "base_link";
  std::vector<std::string> disabledCollisionLinks;
  std::vector<ObstacleSpec> obstacles;
  std::string sourcePath;
};

using PlanningScenePublisher = rclcpp::Publisher<moveit_msgs::msg::PlanningScene>;

inline bool waitForPlanningSceneServices(
  const rclcpp::Node::SharedPtr & node,
  std::chrono::seconds timeout = std::chrono::seconds(8))
{
  auto applyPlanningSceneClient = node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
    "/apply_planning_scene");
  auto getPlanningSceneClient = node->create_client<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene");

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const auto waitSlice = std::min(std::chrono::milliseconds(250), remaining);
    const bool applyReady = applyPlanningSceneClient->wait_for_service(waitSlice);

    if (!applyReady)
    {
      continue;
    }

    const auto afterApply = std::chrono::steady_clock::now();
    if (afterApply >= deadline)
    {
      break;
    }

    const auto remainingAfterApply = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - afterApply);
    const auto getSlice = std::min(std::chrono::milliseconds(250), remainingAfterApply);
    const bool getReady = getPlanningSceneClient->wait_for_service(getSlice);

    if (applyReady && getReady)
    {
      return true;
    }
  }

  RCLCPP_WARN(
    node->get_logger(),
    "Planning scene services were not ready within %.1f s; obstacle publication may be missed if move_group is still starting",
    std::chrono::duration<double>(timeout).count());
  return false;
}

inline bool ensureAllowedCollisionPair(
  moveit_msgs::msg::AllowedCollisionMatrix & allowedCollisionMatrix,
  const std::string & nameA,
  const std::string & nameB)
{
  auto ensureEntry = [&allowedCollisionMatrix](const std::string & name) -> std::size_t
  {
    for (std::size_t index = 0; index < allowedCollisionMatrix.entry_names.size(); ++index)
    {
      if (allowedCollisionMatrix.entry_names[index] == name)
      {
        return index;
      }
    }

    const std::size_t newIndex = allowedCollisionMatrix.entry_names.size();
    allowedCollisionMatrix.entry_names.push_back(name);

    moveit_msgs::msg::AllowedCollisionEntry newRow;
    newRow.enabled.resize(newIndex + 1, false);
    allowedCollisionMatrix.entry_values.push_back(newRow);

    for (auto & row : allowedCollisionMatrix.entry_values)
    {
      row.enabled.resize(newIndex + 1, false);
    }

    return newIndex;
  };

  const auto indexA = ensureEntry(nameA);
  const auto indexB = ensureEntry(nameB);
  allowedCollisionMatrix.entry_values[indexA].enabled[indexB] = true;
  allowedCollisionMatrix.entry_values[indexB].enabled[indexA] = true;
  return true;
}

inline bool loadCurrentAllowedCollisionMatrix(
  const rclcpp::Node::SharedPtr & node,
  moveit_msgs::msg::AllowedCollisionMatrix & allowedCollisionMatrix)
{
  auto getPlanningSceneClient = node->create_client<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene");
  if (!getPlanningSceneClient->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Cannot fetch current planning scene ACM because /get_planning_scene is unavailable");
    return false;
  }

  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  auto future = getPlanningSceneClient->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_WARN(node->get_logger(), "Timed out while fetching current planning scene ACM");
    return false;
  }

  allowedCollisionMatrix = future.get()->scene.allowed_collision_matrix;
  return true;
}

inline bool applyPlanningSceneDiff(
  const rclcpp::Node::SharedPtr & node,
  const moveit_msgs::msg::PlanningScene & planningScene,
  PlanningScenePublisher::SharedPtr & planningScenePublisher)
{
  if (planningScenePublisher == nullptr)
  {
    planningScenePublisher = node->create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(1).reliable().transient_local());
  }

  bool appliedViaService = false;
  auto applyPlanningSceneClient = node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
    "/apply_planning_scene");
  if (applyPlanningSceneClient->wait_for_service(std::chrono::seconds(2)))
  {
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = planningScene;

    auto future = applyPlanningSceneClient->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
      if (future.get()->success)
      {
        appliedViaService = true;
      }
      else
      {
        RCLCPP_WARN(
          node->get_logger(),
          "/apply_planning_scene returned success=false, falling back to /planning_scene topic publish");
      }
    }
    else
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Timed out while applying planning scene diff via service, falling back to topic publish");
    }
  }

  planningScenePublisher->publish(planningScene);
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  planningScenePublisher->publish(planningScene);

  if (!appliedViaService)
  {
    RCLCPP_INFO(node->get_logger(), "Published planning scene diff via /planning_scene topic");
  }

  return true;
}

inline ObstacleVector3 loadVector3Node(const YAML::Node & node)
{
  ObstacleVector3 value;
  if (!node)
  {
    return value;
  }

  value.x = node["x"] ? node["x"].as<double>() : 0.0;
  value.y = node["y"] ? node["y"].as<double>() : 0.0;
  value.z = node["z"] ? node["z"].as<double>() : 0.0;
  return value;
}

inline ObstaclePose loadPoseNode(const YAML::Node & node)
{
  ObstaclePose value;
  if (!node)
  {
    return value;
  }

  value.x = node["x"] ? node["x"].as<double>() : 0.0;
  value.y = node["y"] ? node["y"].as<double>() : 0.0;
  value.z = node["z"] ? node["z"].as<double>() : 0.0;
  value.qx = node["qx"] ? node["qx"].as<double>() : 0.0;
  value.qy = node["qy"] ? node["qy"].as<double>() : 0.0;
  value.qz = node["qz"] ? node["qz"].as<double>() : 0.0;
  value.qw = node["qw"] ? node["qw"].as<double>() : 1.0;

  const double quaternionNorm = std::sqrt(
    (value.qx * value.qx) + (value.qy * value.qy) + (value.qz * value.qz) +
    (value.qw * value.qw));
  if (quaternionNorm <= 1e-6)
  {
    value.qx = 0.0;
    value.qy = 0.0;
    value.qz = 0.0;
    value.qw = 1.0;
  }
  else if (std::abs(quaternionNorm - 1.0) > 1e-6)
  {
    value.qx /= quaternionNorm;
    value.qy /= quaternionNorm;
    value.qz /= quaternionNorm;
    value.qw /= quaternionNorm;
  }

  return value;
}

inline bool loadObstaclesConfig(ObstaclesConfig & config, std::string & errorMessage)
{
  try
  {
    config = ObstaclesConfig{};
    config.sourcePath =
      ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm") +
      "/config/obstacles.yaml";

    const YAML::Node root = YAML::LoadFile(config.sourcePath);
    if (root["planning_frame"])
    {
      config.planningFrame = root["planning_frame"].as<std::string>();
    }

    const YAML::Node disabledCollisionLinksNode = root["disabled_collision_links"];
    if (disabledCollisionLinksNode && disabledCollisionLinksNode.IsSequence())
    {
      config.disabledCollisionLinks.reserve(disabledCollisionLinksNode.size());

      for (std::size_t index = 0; index < disabledCollisionLinksNode.size(); ++index)
      {
        const YAML::Node linkNode = disabledCollisionLinksNode[index];
        if (!linkNode || !linkNode.IsScalar())
        {
          RCLCPP_WARN(
            rclcpp::get_logger("je_arm_pcb_inspection_sm.obstacles"),
            "Skipping disabled collision link entry %zu because it is not a string",
            index);
          continue;
        }

        const auto linkName = linkNode.as<std::string>();
        if (linkName.empty())
        {
          RCLCPP_WARN(
            rclcpp::get_logger("je_arm_pcb_inspection_sm.obstacles"),
            "Skipping disabled collision link entry %zu because it is empty",
            index);
          continue;
        }

        config.disabledCollisionLinks.push_back(linkName);
      }
    }

    const YAML::Node obstaclesNode = root["obstacles"];
    if (!obstaclesNode || !obstaclesNode.IsSequence())
    {
      errorMessage = "obstacles.yaml missing sequence node 'obstacles'";
      return false;
    }

    config.obstacles.reserve(obstaclesNode.size());

    for (std::size_t index = 0; index < obstaclesNode.size(); ++index)
    {
      const YAML::Node obstacleNode = obstaclesNode[index];
      if (!obstacleNode || !obstacleNode.IsMap())
      {
        RCLCPP_WARN(
          rclcpp::get_logger("je_arm_pcb_inspection_sm.obstacles"),
          "Skipping obstacle entry %zu because it is not a map",
          index);
        continue;
      }

      ObstacleSpec obstacle;
      obstacle.id = obstacleNode["id"] ? obstacleNode["id"].as<std::string>() :
        "obstacle_" + std::to_string(index);
      obstacle.enabled = obstacleNode["enabled"] ? obstacleNode["enabled"].as<bool>() : true;
      obstacle.type = obstacleNode["type"] ? obstacleNode["type"].as<std::string>() : "box";
      obstacle.frameId = obstacleNode["frame_id"] ? obstacleNode["frame_id"].as<std::string>() :
        config.planningFrame;
      obstacle.pose = loadPoseNode(obstacleNode["pose"]);
      obstacle.size = loadVector3Node(obstacleNode["size"]);
      obstacle.safetyMargin = loadVector3Node(obstacleNode["safety_margin"]);

      if (obstacle.size.x <= 0.0 || obstacle.size.y <= 0.0 || obstacle.size.z <= 0.0)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("je_arm_pcb_inspection_sm.obstacles"),
          "Skipping obstacle '%s': invalid base size (%.4f, %.4f, %.4f)",
          obstacle.id.c_str(),
          obstacle.size.x,
          obstacle.size.y,
          obstacle.size.z);
        continue;
      }

      if (obstacle.safetyMargin.x < 0.0 || obstacle.safetyMargin.y < 0.0 ||
        obstacle.safetyMargin.z < 0.0)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("je_arm_pcb_inspection_sm.obstacles"),
          "Skipping obstacle '%s': safety_margin must be non-negative",
          obstacle.id.c_str());
        continue;
      }

      config.obstacles.push_back(obstacle);
    }

    errorMessage.clear();
    return true;
  }
  catch (const YAML::BadFile & e)
  {
    config = ObstaclesConfig{};
    errorMessage = std::string("cannot read obstacles file: ") + e.what();
    return false;
  }
  catch (const YAML::ParserException & e)
  {
    config = ObstaclesConfig{};
    errorMessage = std::string("invalid obstacles yaml: ") + e.what();
    return false;
  }
  catch (const std::exception & e)
  {
    config = ObstaclesConfig{};
    errorMessage = e.what();
    return false;
  }
}

inline geometry_msgs::msg::Pose toGeometryPose(const ObstaclePose & pose)
{
  geometry_msgs::msg::Pose geometryPose;
  geometryPose.position.x = pose.x;
  geometryPose.position.y = pose.y;
  geometryPose.position.z = pose.z;
  geometryPose.orientation.x = pose.qx;
  geometryPose.orientation.y = pose.qy;
  geometryPose.orientation.z = pose.qz;
  geometryPose.orientation.w = pose.qw;
  return geometryPose;
}

inline bool publishPlanningSceneObstacles(
  const rclcpp::Node::SharedPtr & node,
  PlanningScenePublisher::SharedPtr & planningScenePublisher)
{
  if (node == nullptr)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("je_arm_pcb_inspection_sm.obstacles"),
      "Cannot publish planning scene obstacles because node is null");
    return false;
  }

  ObstaclesConfig config;
  std::string errorMessage;
  if (!loadObstaclesConfig(config, errorMessage))
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Skipping planning scene obstacle publish: failed to load %s (%s)",
      config.sourcePath.empty() ? "config/obstacles.yaml" : config.sourcePath.c_str(),
      errorMessage.c_str());
    return false;
  }

  waitForPlanningSceneServices(node);

  moveit_msgs::msg::PlanningScene planningScene;
  planningScene.is_diff = true;

  if (!config.disabledCollisionLinks.empty())
  {
    if (!loadCurrentAllowedCollisionMatrix(node, planningScene.allowed_collision_matrix))
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Skipping disabled_collision_links because the current MoveIt ACM could not be loaded safely");
    }
  }

  for (const auto & obstacle : config.obstacles)
  {
    if (!obstacle.enabled)
    {
      RCLCPP_INFO(node->get_logger(), "Skipping disabled obstacle '%s'", obstacle.id.c_str());
      continue;
    }

    if (obstacle.type != "box")
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Skipping obstacle '%s': unsupported type '%s'",
        obstacle.id.c_str(),
        obstacle.type.c_str());
      continue;
    }

    const double sizeX = obstacle.size.x + (2.0 * obstacle.safetyMargin.x);
    const double sizeY = obstacle.size.y + (2.0 * obstacle.safetyMargin.y);
    const double sizeZ = obstacle.size.z + (2.0 * obstacle.safetyMargin.z);

    if (sizeX <= 0.0 || sizeY <= 0.0 || sizeZ <= 0.0)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Skipping obstacle '%s': non-positive dimensions after safety margin (%.4f, %.4f, %.4f)",
        obstacle.id.c_str(),
        sizeX,
        sizeY,
        sizeZ);
      continue;
    }

    moveit_msgs::msg::CollisionObject collisionObject;
    collisionObject.id = obstacle.id;
    collisionObject.header.frame_id = obstacle.frameId.empty() ? config.planningFrame : obstacle.frameId;
    collisionObject.header.stamp = node->now();
    collisionObject.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {sizeX, sizeY, sizeZ};

    collisionObject.primitives.push_back(primitive);
    collisionObject.primitive_poses.push_back(toGeometryPose(obstacle.pose));
    planningScene.world.collision_objects.push_back(collisionObject);

    for (const auto & linkName : config.disabledCollisionLinks)
    {
      if (planningScene.allowed_collision_matrix.entry_names.empty() &&
        planningScene.allowed_collision_matrix.entry_values.empty())
      {
        break;
      }

      ensureAllowedCollisionPair(planningScene.allowed_collision_matrix, linkName, collisionObject.id);
      RCLCPP_INFO(
        node->get_logger(),
        "Merged MoveIt ACM override for pair '%s' <-> '%s'",
        linkName.c_str(),
        collisionObject.id.c_str());
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Planning scene obstacle '%s' loaded from %s in frame '%s' with size=(%.4f, %.4f, %.4f)",
      obstacle.id.c_str(),
      config.sourcePath.c_str(),
      collisionObject.header.frame_id.c_str(),
      sizeX,
      sizeY,
      sizeZ);
  }

  if (
    planningScene.world.collision_objects.empty() &&
    planningScene.allowed_collision_matrix.entry_names.empty())
  {
    RCLCPP_WARN(
      node->get_logger(),
      "No enabled planning scene obstacles or collision overrides were published");
    return false;
  }

  if (!applyPlanningSceneDiff(node, planningScene, planningScenePublisher))
  {
    return false;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Published %zu planning scene obstacles and merged %zu ACM rows",
    planningScene.world.collision_objects.size(),
    planningScene.allowed_collision_matrix.entry_names.size());
  RCLCPP_INFO(
    je_arm_pcb_inspection_sm::log_utils::bizLogger(),
    "[%s] STATIC_OBSTACLES published: count=%zu ids=%s",
    je_arm_pcb_inspection_sm::log_utils::bjtNowString().c_str(),
    planningScene.world.collision_objects.size(),
    planningScene.world.collision_objects.empty() ? "<none>" :
    planningScene.world.collision_objects.front().id.c_str());
  return true;
}

}  // namespace je_arm_pcb_inspection_sm::utils