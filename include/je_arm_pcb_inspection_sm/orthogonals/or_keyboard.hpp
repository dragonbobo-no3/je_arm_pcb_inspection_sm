// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <chrono>
#include <cl_keyboard/cl_keyboard.hpp>
#include <smacc2/smacc.hpp>

#include "je_arm_pcb_inspection_sm/components/cp_business_key_mapper.hpp"
#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"

namespace je_arm_pcb_inspection_sm {
using namespace std::chrono_literals;

class OrKeyboard : public smacc2::Orthogonal<OrKeyboard> {
public:
  void onInitialize() override {
    auto keyboardClient = this->createClient<cl_keyboard::ClKeyboard>();
    keyboardClient->createComponent<je_arm_pcb_inspection_sm::CpBusinessKeyMapper>();
    keyboardClient->createComponent<je_arm_pcb_inspection_sm::CpTopLevelFlow>();
  }
};
} // namespace je_arm_pcb_inspection_sm
