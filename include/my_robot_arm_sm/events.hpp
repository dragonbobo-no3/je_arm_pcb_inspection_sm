#pragma once

#include <smacc2/smacc.hpp>

namespace my_robot_arm_sm
{

// ========== 顶层状态事件 ==========

// IDLE -> WAIT_RESOURCES
struct EvStartWork : sc::event<EvStartWork> {};

// WAIT_RESOURCES -> WORK
// WORK -> WORK (循环)
struct EvCanWork : sc::event<EvCanWork> {};

// WAIT_RESOURCES -> IDLE (超时)
struct EvWaitTimeout : sc::event<EvWaitTimeout> {};

// WAIT_RESOURCES / WORK / BACK -> PAUSE
struct EvPauseRequested : sc::event<EvPauseRequested> {};

// WORK -> BACK
struct EvResourcesUnavailable : sc::event<EvResourcesUnavailable> {};

// BACK -> WAIT_RESOURCES
struct EvBackDone : sc::event<EvBackDone> {};

// PAUSE -> WAIT_RESOURCES / WORK (根据resume_state_id)
struct EvKeyResume : sc::event<EvKeyResume> {};

// PAUSE -> BACK
struct EvKeyBack : sc::event<EvKeyBack> {};

// PAUSE -> BACK
struct EvFaultToBack : sc::event<EvFaultToBack> {};

}  // namespace my_robot_arm_sm
