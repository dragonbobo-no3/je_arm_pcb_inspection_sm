#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include <cl_keyboard/components/cp_keyboard_listener_1.hpp>

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
    cl_keyboard::components::CpKeyboardListener1 * keyboardListener;
    this->requiresComponent(keyboardListener);
    keyboardListener->OnKeyPress(&CpBusinessKeyMapper::onKeyPress, this);
    RCLCPP_INFO(log_utils::bizLogger(), "[%s] KEY_MAPPER ready", log_utils::bjtNowString().c_str());
  }

  void onKeyPress(char key)
  {
    auto * currentState = this->getStateMachine()->getCurrentState();
    if (currentState == nullptr)
    {
      return;
    }

    const std::string stateName = currentState->getClassName();

    auto setPauseReasonFromState = [&]() {
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPauseReason),
        std::string("[") + log_utils::bjtNowString() + "] 手动暂停: key='p', state=" + stateName);
    };

    if (stateName.find("StIdle") != std::string::npos)
    {
      if (key == 's')
      {
        this->postEvent<EvStartWork>();
      }
      return;
    }

    if (stateName.find("StWaitResources") != std::string::npos)
    {
      if (key == 'w')
      {
        this->postEvent<EvCanWork>();
      }
      else if (key == 'p')
      {
        setPauseReasonFromState();
        this->postEvent<EvPauseRequested>();
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
      else if (stateName.find("StInspect") != std::string::npos && key == 'n')
      {
        this->postEvent<EvInspectDone>();
      }
      else if (stateName.find("StSelectBin") != std::string::npos && key == 'n')
      {
        this->postEvent<EvBinSelected>();
      }
      else if (stateName.find("StPlace") != std::string::npos && key == 'n')
      {
        this->postEvent<EvPlaceDone>();
      }
      else if (stateName.find("StPlaceDecision") != std::string::npos && key == 'w')
      {
        this->postEvent<EvCanWork>();
      }
      else if (stateName.find("StPlaceDecision") != std::string::npos && key == 'u')
      {
        this->postEvent<EvResourcesUnavailable>();
      }
      else if (key == 'p')
      {
        this->postEvent<EvPauseRequested>();
      }
      else if (key == 'u')
      {
        this->postEvent<EvResourcesUnavailable>();
      }
      return;
    }

    if (stateName.find("StBack") != std::string::npos)
    {
      if (key == 'p')
      {
        setPauseReasonFromState();
        this->postEvent<EvPauseRequested>();
      }
      return;
    }

    if (stateName.find("StDelay") != std::string::npos)
    {
      if (key == 'p')
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
};

}  // namespace je_arm_pcb_inspection_sm