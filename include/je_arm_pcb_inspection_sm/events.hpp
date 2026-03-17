#pragma once

#include <smacc2/smacc.hpp>

namespace je_arm_pcb_inspection_sm
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

// PAUSE 恢复路由中间事件
struct EvResumeToWaitResources : sc::event<EvResumeToWaitResources> {};
struct EvResumeToWork : sc::event<EvResumeToWork> {};
struct EvResumeToBack : sc::event<EvResumeToBack> {};

// WORK 主干阶段事件
struct EvPickDone : sc::event<EvPickDone> {};
struct EvInspectDone : sc::event<EvInspectDone> {};
struct EvBinSelected : sc::event<EvBinSelected> {};
struct EvPlaceDone : sc::event<EvPlaceDone> {};

// PICK 内部阶段事件
struct EvAtPregrasp : sc::event<EvAtPregrasp> {};
struct EvGripperOpened : sc::event<EvGripperOpened> {};
struct EvAtGraspDepth : sc::event<EvAtGraspDepth> {};
struct EvGripperClosed : sc::event<EvGripperClosed> {};
struct EvLifted : sc::event<EvLifted> {};

// WORK 内部恢复路由事件
struct EvWorkResumeToPick : sc::event<EvWorkResumeToPick> {};
struct EvWorkResumeToInspect : sc::event<EvWorkResumeToInspect> {};
struct EvWorkResumeToSelectBin : sc::event<EvWorkResumeToSelectBin> {};
struct EvWorkResumeToPlace : sc::event<EvWorkResumeToPlace> {};
struct EvWorkResumeToPlaceDecision : sc::event<EvWorkResumeToPlaceDecision> {};

// PICK 内部恢复路由事件
struct EvPickResumeToLPregrasp : sc::event<EvPickResumeToLPregrasp> {};  // fallback → P1
struct EvPickResumeToLPregraspP1 : sc::event<EvPickResumeToLPregraspP1> {};
struct EvPickResumeToLPregraspP2 : sc::event<EvPickResumeToLPregraspP2> {};
struct EvPickResumeToLPregraspP3 : sc::event<EvPickResumeToLPregraspP3> {};
struct EvPickResumeToGripperOpen : sc::event<EvPickResumeToGripperOpen> {};
struct EvPickResumeToCartesianDown : sc::event<EvPickResumeToCartesianDown> {};
struct EvPickResumeToGripperClose : sc::event<EvPickResumeToGripperClose> {};
struct EvPickResumeToCartesianUp : sc::event<EvPickResumeToCartesianUp> {};
struct EvPickResumeToLRetreat : sc::event<EvPickResumeToLRetreat> {};

}  // namespace je_arm_pcb_inspection_sm
