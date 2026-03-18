// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// ORTHOGONALS
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_keyboard.hpp"

#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/client_behaviors.hpp>
#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <smacc2/client_behaviors/cb_wait_topic_message.hpp>
#include <smacc2/smacc_updatable.hpp>

// EVENTS (must be after smacc2/smacc.hpp)
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

using namespace cl_moveit2z;
using namespace cl_keyboard;
using namespace smacc2::default_events;

namespace je_arm_pcb_inspection_sm
{

// STATES - Forward declarations
struct StIdle;

//====================================
// STATE_MACHINE
//====================================

struct SmJeArmPcbInspection : public smacc2::SmaccStateMachineBase<SmJeArmPcbInspection, StIdle>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override 
  { 
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWaitResourcesState));
    this->setGlobalSMData(std::string(sm_data::kResumeFromPause), false);
    this->setGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstatePick));
    this->setGlobalSMData(std::string(sm_data::kPickResumeFromPause), false);
    this->setGlobalSMData(std::string(sm_data::kPickResumeSubstateId), std::string(sm_data::kPickSubstateLPregrasp));
    this->setGlobalSMData(std::string(sm_data::kPcbPresent), true);
    this->setGlobalSMData(std::string(sm_data::kLeftSlotFree), true);
    this->setGlobalSMData(std::string(sm_data::kRightSlotFree), true);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetFrameId), std::string("base_link"));
    this->setGlobalSMData(std::string(sm_data::kPcbTargetX), -0.38223850462518716);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetY), 0.17516332117030306);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetZ), 0.3253017070504734);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetQx), 0.577391798319681);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetQy), 0.4299463000132024);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetQz), -0.5282853654249783);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetQw), -0.4501993591909244);
    this->setGlobalSMData(std::string(sm_data::kWaitTimeoutSec), 10.0);
    this->setGlobalSMData(std::string(sm_data::kWorkCycleSec), 2.0);
    this->setGlobalSMData(std::string(sm_data::kBackHomeSec), 1.5);

    this->setTransitionCallback(
      [this](const smacc2::introspection::SmaccTransitionInfo & transitionInfo)
      {
        const std::string source =
          transitionInfo.sourceState ? transitionInfo.sourceState->toShortName() : "<unknown>";
        const std::string target =
          transitionInfo.destinyState ? transitionInfo.destinyState->toShortName() : "<unknown>";
        const std::string event =
          transitionInfo.eventInfo ? transitionInfo.eventInfo->getEventTypeName() : "<unknown>";

        RCLCPP_INFO(
          log_utils::bizLogger(),
          "[%s] TRANSITION %s --(%s)--> %s | %s",
          log_utils::bjtNowString().c_str(),
          source.c_str(),
          event.c_str(),
          target.c_str(),
          log_utils::formatArmSnapshot(*this).c_str());
      });

    this->createOrthogonal<OrArm>(); 
    this->createOrthogonal<OrKeyboard>();
  }
};

}  // namespace je_arm_pcb_inspection_sm

// ====================================
// STATES - MUST be included AFTER SM definition
// ====================================

#include "states/st_idle.hpp"
#include "states/st_wait_resources.hpp"
#include "states/st_work.hpp"
#include "states/st_back.hpp"
#include "states/st_pause_resume_router.hpp"
#include "states/st_pause.hpp"
