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
#include "my_robot_arm_sm/orthogonals/or_arm.hpp"
#include "my_robot_arm_sm/orthogonals/or_keyboard.hpp"

#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/client_behaviors.hpp>
#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <smacc2/client_behaviors/cb_wait_topic_message.hpp>
#include <smacc2/smacc_updatable.hpp>

// EVENTS (must be after smacc2/smacc.hpp)
#include "my_robot_arm_sm/events.hpp"
#include "my_robot_arm_sm/sm_data.hpp"

using namespace cl_moveit2z;
using namespace cl_keyboard;
using namespace smacc2::default_events;

namespace my_robot_arm_sm
{

// STATES - Forward declarations
struct StIdle;

//====================================
// STATE_MACHINE
//====================================

struct SmMyRobotArm : public smacc2::SmaccStateMachineBase<SmMyRobotArm, StIdle>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override 
  { 
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWaitResourcesState));
    this->setGlobalSMData(std::string(sm_data::kResumeFromPause), false);
    this->setGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstatePick));
    this->setGlobalSMData(std::string(sm_data::kPcbPresent), true);
    this->setGlobalSMData(std::string(sm_data::kLeftSlotFree), true);
    this->setGlobalSMData(std::string(sm_data::kRightSlotFree), true);
    this->setGlobalSMData(std::string(sm_data::kWaitTimeoutSec), 10.0);
    this->setGlobalSMData(std::string(sm_data::kWorkCycleSec), 2.0);
    this->setGlobalSMData(std::string(sm_data::kBackHomeSec), 1.5);

    this->createOrthogonal<OrArm>(); 
    this->createOrthogonal<OrKeyboard>();
  }
};

}  // namespace my_robot_arm_sm

// ====================================
// STATES - MUST be included AFTER SM definition
// ====================================

#include "states/st_idle.hpp"
#include "states/st_wait_resources.hpp"
#include "states/st_work.hpp"
#include "states/st_back.hpp"
#include "states/st_pause_resume_router.hpp"
#include "states/st_pause.hpp"
