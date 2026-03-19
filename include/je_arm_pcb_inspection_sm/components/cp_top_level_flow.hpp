#pragma once

#include <cmath>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <je_arm_pcb_inspection_sm/msg/pcb_detection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

class CpTopLevelFlow : public smacc2::ISmaccComponent
{
public:
  CpTopLevelFlow() = default;
  virtual ~CpTopLevelFlow() = default;

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    auto node = this->getNode();

    pcbDetectionSub_ = node->create_subscription<je_arm_pcb_inspection_sm::msg::PcbDetection>(
      pcbDetectionTopic_,
      rclcpp::QoS(10),
      [this](const je_arm_pcb_inspection_sm::msg::PcbDetection::SharedPtr msg)
      {
        std::scoped_lock<std::mutex> lock(dataMutex_);
        if (msg->present && isPoseValid(msg->pose.pose))
        {
          latestPcbPose_ = msg->pose;
          hasValidPcbPose_ = true;
        }
        else
        {
          hasValidPcbPose_ = false;
        }
      });

    RCLCPP_INFO(
      getLogger(),
      "[CpTopLevelFlow] Subscribed: pcb_detection='%s' (present+pose)",
      pcbDetectionTopic_.c_str());
  }

  void setResumeTarget(const char * target)
  {
    this->getStateMachine()->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(target));
  }

  bool isWorkReady()
  {
    syncTopicDataToBlackboard();

    const bool pcbPresent = getBoolData(sm_data::kPcbPresent, true);
    const bool leftSlotFree = getBoolData(sm_data::kLeftSlotFree, true);
    const bool rightSlotFree = getBoolData(sm_data::kRightSlotFree, true);
    return pcbPresent && (leftSlotFree || rightSlotFree);
  }

  void setPcbPresent(bool value)
  {
    this->getStateMachine()->setGlobalSMData(std::string(sm_data::kPcbPresent), value);
  }

  double waitTimeoutSeconds()
  {
    return getDoubleData(sm_data::kWaitTimeoutSec, 10.0);
  }

  double workCycleSeconds()
  {
    return getDoubleData(sm_data::kWorkCycleSec, 2.0);
  }

  double backHomeSeconds()
  {
    return getDoubleData(sm_data::kBackHomeSec, 1.5);
  }

private:
  void syncTopicDataToBlackboard()
  {
    std::scoped_lock<std::mutex> lock(dataMutex_);

    this->getStateMachine()->setGlobalSMData(
      std::string(sm_data::kPcbPresent), hasValidPcbPose_);

    if (hasValidPcbPose_)
    {
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetFrameId), latestPcbPose_.header.frame_id);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetX), latestPcbPose_.pose.position.x);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetY), latestPcbPose_.pose.position.y);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetZ), latestPcbPose_.pose.position.z);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetQx), latestPcbPose_.pose.orientation.x);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetQy), latestPcbPose_.pose.orientation.y);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetQz), latestPcbPose_.pose.orientation.z);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetQw), latestPcbPose_.pose.orientation.w);
    }
  }

  bool isPoseValid(const geometry_msgs::msg::Pose & pose) const
  {
    const bool positionFinite =
      std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z);

    const bool orientationFinite =
      std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
      std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);

    const double qnorm2 =
      pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;

    return positionFinite && orientationFinite && qnorm2 > 1e-12;
  }

  bool getBoolData(const char * key, bool fallback)
  {
    bool value = fallback;
    this->getStateMachine()->getGlobalSMData(std::string(key), value);
    return value;
  }

  double getDoubleData(const char * key, double fallback)
  {
    double value = fallback;
    this->getStateMachine()->getGlobalSMData(std::string(key), value);
    return value;
  }

  std::mutex dataMutex_;
  bool hasValidPcbPose_{false};
  geometry_msgs::msg::PoseStamped latestPcbPose_;

  std::string pcbDetectionTopic_{"/vision/pcb_detection"};

  rclcpp::Subscription<je_arm_pcb_inspection_sm::msg::PcbDetection>::SharedPtr pcbDetectionSub_;
};

}  // namespace je_arm_pcb_inspection_sm