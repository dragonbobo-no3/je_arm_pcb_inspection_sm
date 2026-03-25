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

#pragma once

#include <chrono>

#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/components/cp_motion_planner.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>
#include <cl_moveit2z/components/cp_trajectory_history.hpp>
#include <cl_moveit2z/components/cp_grasping_objects.hpp>

#include "cl_ros2_timer/cl_ros2_timer.hpp"
#include "smacc2/smacc.hpp"

namespace je_arm_pcb_inspection_sm
{
using namespace std::chrono_literals;

class OrLeftArm : public smacc2::Orthogonal<OrLeftArm>
{
public:
  void onInitialize() override
  {
    auto move_group_client = this->createClient<cl_moveit2z::ClMoveit2z>("left_arm");
    move_group_client->createComponent<cl_moveit2z::CpMotionPlanner>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryExecutor>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryHistory>();
    auto graspingComponent = move_group_client->createComponent<cl_moveit2z::CpGraspingComponent>();

    graspingComponent->gripperLink_ = "Link17";
    graspingComponent->createGraspableBox("virtualBox", 0, 0.5, 0.5, 0.1, 0.1, 0.1);
  }
};

class OrRightArm : public smacc2::Orthogonal<OrRightArm>
{
public:
  void onInitialize() override
  {
    auto move_group_client = this->createClient<cl_moveit2z::ClMoveit2z>("right_arm");
    move_group_client->createComponent<cl_moveit2z::CpMotionPlanner>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryExecutor>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryHistory>();
    auto graspingComponent = move_group_client->createComponent<cl_moveit2z::CpGraspingComponent>();

    graspingComponent->gripperLink_ = "Link27";
    graspingComponent->createGraspableBox("virtualBox", 0, 0.5, 0.5, 0.1, 0.1, 0.1);
  }
};

class OrBothArms : public smacc2::Orthogonal<OrBothArms>
{
public:
  void onInitialize() override
  {
    auto move_group_client = this->createClient<cl_moveit2z::ClMoveit2z>("both_arms");
    move_group_client->createComponent<cl_moveit2z::CpMotionPlanner>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryExecutor>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryHistory>();
  }
};

class OrGripper : public smacc2::Orthogonal<OrGripper>
{
public:
  void onInitialize() override {}
};

using OrArm = OrLeftArm;
}  // namespace je_arm_pcb_inspection_sm
