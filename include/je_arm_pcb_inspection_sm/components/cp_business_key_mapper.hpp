#pragma once

#include <chrono>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <je_arm_pcb_inspection_sm/msg/pcb_detection.hpp>
#include <je_arm_pcb_inspection_sm/msg/place_slot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>
#include <yaml-cpp/yaml.h>

#include <cl_keyboard/components/cp_keyboard_listener_1.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

class CpBusinessKeyMapper : public smacc2::ISmaccComponent
{
public:
  CpBusinessKeyMapper() = default;
  virtual ~CpBusinessKeyMapper() = default;

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    RCLCPP_WARN(getLogger(), "CpBusinessKeyMapper::onComponentInitialization - STARTING");
    cl_keyboard::components::CpKeyboardListener1 * keyboardListener;
    this->requiresComponent(keyboardListener);
    RCLCPP_WARN(getLogger(), "CpBusinessKeyMapper - got keyboard listener, registering callback");
    keyboardListener->OnKeyPress(&CpBusinessKeyMapper::onKeyPress, this);

    auto node = this->getNode();
    pcbDetectionPub_ =
      node->create_publisher<je_arm_pcb_inspection_sm::msg::PcbDetection>(
        pcbDetectionTopic_, rclcpp::QoS(10));
    placeSlotPub_ =
      node->create_publisher<je_arm_pcb_inspection_sm::msg::PlaceSlot>(
        placeSlotTopic_, rclcpp::QoS(10));

    RCLCPP_INFO(log_utils::bizLogger(), "[%s] KEY_MAPPER ready", log_utils::bjtNowString().c_str());
  }

  void onKeyPress(char key)
  {
    auto * currentState = this->getStateMachine()->getCurrentState();
    if (currentState == nullptr)
    {
      RCLCPP_WARN(getLogger(), "CpBusinessKeyMapper::onKeyPress - currentState is nullptr");
      return;
    }

    const std::string stateName = currentState->getClassName();

    auto setPauseReasonFromState = [&]() {
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPauseReason),
        std::string("[") + log_utils::bjtNowString() + "] 手动暂停: key='p', state=" + stateName);
    };

    auto tryPostPauseRequested = [&](bool setReason) {
      if (key != 'p')
      {
        return false;
      }

      if (isPauseDebounced())
      {
        return true;
      }

      if (setReason)
      {
        setPauseReasonFromState();
      }

      this->postEvent<EvPauseRequested>();
      return true;
    };

    if (stateName.find("StIdle") != std::string::npos)
    {
      RCLCPP_WARN(getLogger(), "  -> State matched: StIdle");
      if (key == 's' || key == 'S')
      {
        RCLCPP_WARN(getLogger(), "  -> Key matched: 's' or 'S', posting EvStartWork");
        this->postEvent<EvStartWork>();
      }
      else if (key == 't' || key == 'T')
      {
        RCLCPP_WARN(getLogger(), "  -> Key matched: 't' or 'T', posting EvTestGripper");
        this->postEvent<EvTestGripper>();
      }
      else
      {
        RCLCPP_WARN(getLogger(), "  -> Key not matched: got '%c', expected 's' or 't'", key);
      }
      return;
    }

    if (stateName.find("StWaitResources") != std::string::npos)
    {
      if (key == 'w')
      {
        publishPcbDetection(true);
      }
      else if (key == 'u')
      {
        publishPcbDetection(false);
      }
      else if (key == 'h')
      {
        publishFreePlaceSlotFromYaml();
      }
      else if (key == 'p')
      {
        (void)tryPostPauseRequested(true);
      }
      else if (key == 'i')
      {
        this->postEvent<EvBackToIdleRequested>();
      }
      return;
    }

    if (stateName.find("StActivate") != std::string::npos)
    {
      if (key == 'p')
      {
        cl_moveit2z::CpTrajectoryExecutor * trajectoryExecutor = nullptr;
        this->requiresComponent(trajectoryExecutor, false);
        if (trajectoryExecutor != nullptr)
        {
          trajectoryExecutor->cancel();
        }

        (void)tryPostPauseRequested(true);
      }
      return;
    }

    if (
      stateName.find("StWork") != std::string::npos ||
      stateName.find("StPick") != std::string::npos ||
      stateName.find("StLPregrasp") != std::string::npos ||
      stateName.find("StGripperOpen") != std::string::npos ||
      stateName.find("StCartesianDown") != std::string::npos ||
      stateName.find("StGripperClose") != std::string::npos ||
      stateName.find("StCartesianUp") != std::string::npos ||
      stateName.find("StLRetreat") != std::string::npos ||
      stateName.find("StInspect") != std::string::npos ||
      stateName.find("StSelectBin") != std::string::npos ||
      stateName.find("StPlace") != std::string::npos)
    {
      if (stateName.find("StLPregrasp") != std::string::npos && key == 'n')
      {
        this->postEvent<EvAtPregrasp>();
      }
      else if (stateName.find("StGripperOpen") != std::string::npos && key == 'n')
      {
        this->postEvent<EvGripperOpened>();
      }
      else if (stateName.find("StCartesianDown") != std::string::npos && key == 'n')
      {
        this->postEvent<EvAtGraspDepth>();
      }
      else if (stateName.find("StGripperClose") != std::string::npos && key == 'n')
      {
        this->postEvent<EvGripperClosed>();
      }
      else if (stateName.find("StCartesianUp") != std::string::npos && key == 'n')
      {
        this->postEvent<EvLifted>();
      }
      else if (stateName.find("StLRetreat") != std::string::npos && key == 'n')
      {
        this->postEvent<EvPickDone>();
      }
      else if (stateName.find("StPick") != std::string::npos && key == 'n')
      {
        this->postEvent<EvAtPregrasp>();
      }
      else if (stateName.find("StSelectBin") != std::string::npos && key == 'n')
      {
        this->postEvent<EvBinSelected>();
      }
      else if (stateName.find("StPlaceGripperOpen") != std::string::npos && key == 'n')
      {
        this->postEvent<EvPlaceReleased>();
      }
      else if (stateName.find("StPlaceGripperClose") != std::string::npos && key == 'n')
      {
        this->postEvent<EvPlaceDone>();
      }
      else if (key == 'p')
      {
        (void)tryPostPauseRequested(false);
      }
      else if (key == 'u')
      {
        publishPcbDetection(false);
      }
      else if (key == 'h')
      {
        publishFreePlaceSlotFromYaml();
      }
      return;
    }

    if (stateName.find("StBack") != std::string::npos)
    {
      if (key == 'p')
      {
        cl_moveit2z::CpTrajectoryExecutor * trajectoryExecutor = nullptr;
        this->requiresComponent(trajectoryExecutor, false);
        if (trajectoryExecutor != nullptr)
        {
          trajectoryExecutor->cancel();
        }
      }
      else if (key == 'u')
      {
        publishPcbDetection(false);
        return;
      }
      else if (key == 'h')
      {
        publishFreePlaceSlotFromYaml();
        return;
      }

      (void)tryPostPauseRequested(true);
      return;
    }

    if (stateName.find("StDelay") != std::string::npos)
    {
      if (key == 'p' && !isPauseDebounced())
      {
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPauseReason),
          std::string("[") + log_utils::bjtNowString() + "] 手动暂停: key='p', state=" + stateName);
        this->postEvent<EvPauseRequested>();
      }
      return;
    }

    if (stateName.find("StPause") != std::string::npos)
    {
      if (key == 'r')
      {
        this->postEvent<EvKeyResume>();
      }
      else if (key == 'b')
      {
        this->postEvent<EvKeyBack>();
      }
      else if (key == 'f')
      {
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPauseReason),
          std::string("[") + log_utils::bjtNowString() + "] 故障转BACK: key='f', state=" + stateName);
        this->postEvent<EvFaultToBack>();
      }
    }
  }

private:
  bool isPauseDebounced()
  {
    const auto now = std::chrono::steady_clock::now();
    if (lastPauseRequestTime_.time_since_epoch().count() != 0)
    {
      const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPauseRequestTime_).count();
      if (elapsedMs < 500)
      {
        return true;
      }
    }

    lastPauseRequestTime_ = now;
    return false;
  }

  void publishPcbDetection(bool present)
  {
    je_arm_pcb_inspection_sm::msg::PcbDetection detectionMsg;
    detectionMsg.present = present;
    detectionMsg.pose.header.stamp = this->getNode()->now();
    detectionMsg.pose.header.frame_id = "base_link";
    detectionMsg.pose.pose.orientation.w = 1.0;

    if (present)
    {
      // Load pick coordinates from YAML as default
      try
      {
        const std::string packageShareDir =
          ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm");
        const std::string yamlPath =
          packageShareDir + "/config/move_group_client/cartesian_states/pick.yaml";

        YAML::Node root = YAML::LoadFile(yamlPath);

        detectionMsg.pose.header.frame_id =
          root["frame_id"] ? root["frame_id"].as<std::string>() : std::string("base_link");
        detectionMsg.pose.pose.position.x = root["x"].as<double>();
        detectionMsg.pose.pose.position.y = root["y"].as<double>();
        detectionMsg.pose.pose.position.z = root["z"].as<double>();
        detectionMsg.pose.pose.orientation.x = root["qx"].as<double>();
        detectionMsg.pose.pose.orientation.y = root["qy"].as<double>();
        detectionMsg.pose.pose.orientation.z = root["qz"].as<double>();
        detectionMsg.pose.pose.orientation.w = root["qw"].as<double>();
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(
          getLogger(),
          "Failed to load pick.yaml: %s, using default values",
          e.what());
        detectionMsg.pose.pose.position.x = -0.273;
        detectionMsg.pose.pose.position.y = 0.0;
        detectionMsg.pose.pose.position.z = 0.412;
        detectionMsg.pose.pose.orientation.x = 0.0;
        detectionMsg.pose.pose.orientation.y = -0.764357;
        detectionMsg.pose.pose.orientation.z = 0.0;
        detectionMsg.pose.pose.orientation.w = 0.644794;
      }
    }

    if (pcbDetectionPub_ != nullptr)
    {
      pcbDetectionPub_->publish(detectionMsg);
    }

    if (present)
    {
      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] KEY 'w': published pcb_detection present=true pose=(%.4f, %.4f, %.4f)",
        log_utils::bjtNowString().c_str(),
        detectionMsg.pose.pose.position.x,
        detectionMsg.pose.pose.position.y,
        detectionMsg.pose.pose.position.z);
    }
    else
    {
      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] KEY 'u': published pcb_detection present=false",
        log_utils::bjtNowString().c_str());
    }
  }

  void publishFreePlaceSlotFromYaml()
  {
    try
    {
      const std::string packageShareDir =
        ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm");
      const std::string yamlPath =
        packageShareDir + "/config/move_group_client/cartesian_states/place.yaml";

      YAML::Node root = YAML::LoadFile(yamlPath);

      je_arm_pcb_inspection_sm::msg::PlaceSlot slotMsg;
      slotMsg.free = true;
      slotMsg.pose.header.stamp = this->getNode()->now();
      slotMsg.pose.header.frame_id =
        root["frame_id"] ? root["frame_id"].as<std::string>() : std::string("base_link");
      slotMsg.pose.pose.position.x = root["x"].as<double>();
      slotMsg.pose.pose.position.y = root["y"].as<double>();
      slotMsg.pose.pose.position.z = root["z"].as<double>();
      slotMsg.pose.pose.orientation.x = root["qx"].as<double>();
      slotMsg.pose.pose.orientation.y = root["qy"].as<double>();
      slotMsg.pose.pose.orientation.z = root["qz"].as<double>();
      slotMsg.pose.pose.orientation.w = root["qw"].as<double>();

      if (placeSlotPub_ != nullptr)
      {
        placeSlotPub_->publish(slotMsg);
      }

      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] KEY 'h': published place_slot free=true pose=(%.4f, %.4f, %.4f)",
        log_utils::bjtNowString().c_str(),
        slotMsg.pose.pose.position.x,
        slotMsg.pose.pose.position.y,
        slotMsg.pose.pose.position.z);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        log_utils::bizLogger(),
        "[%s] KEY 'h': failed to load/publish place slot pose: %s",
        log_utils::bjtNowString().c_str(),
        e.what());
    }
  }

  std::string pcbDetectionTopic_{"/vision/pcb_detection"};
  std::string placeSlotTopic_{"/vision/place_slot_detection"};
  rclcpp::Publisher<je_arm_pcb_inspection_sm::msg::PcbDetection>::SharedPtr pcbDetectionPub_;
  rclcpp::Publisher<je_arm_pcb_inspection_sm::msg::PlaceSlot>::SharedPtr placeSlotPub_;
  std::chrono::steady_clock::time_point lastPauseRequestTime_{};
};

}  // namespace je_arm_pcb_inspection_sm