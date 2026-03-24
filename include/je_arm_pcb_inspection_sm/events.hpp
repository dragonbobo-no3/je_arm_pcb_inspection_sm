#pragma once

#include <smacc2/smacc.hpp>

namespace je_arm_pcb_inspection_sm
{

// ========== 顶层状态事件 ==========

// IDLE -> WAIT_RESOURCES
struct EvStartWork : sc::event<EvStartWork> {};

// IDLE -> TEST_GRIPPER (用于测试夹爪功能)
struct EvTestGripper : sc::event<EvTestGripper> {};

// WAIT_RESOURCES -> WORK
// WORK -> WORK (循环)
struct EvCanWork : sc::event<EvCanWork> {};

// WAIT_RESOURCES -> IDLE (超时)
struct EvWaitTimeout : sc::event<EvWaitTimeout> {};

// WAIT_RESOURCES / WORK / BACK -> PAUSE
struct EvPauseRequested : sc::event<EvPauseRequested> {};

// ANY_STATE -> DELAY (shared reusable timed hold)
struct EvDelayRequested : sc::event<EvDelayRequested> {};

// WORK -> BACK
struct EvResourcesUnavailable : sc::event<EvResourcesUnavailable> {};

// BACK -> WAIT_RESOURCES
struct EvBackDone : sc::event<EvBackDone> {};

// WAIT_RESOURCES -> BACK_TO_IDLE (reverse activate)
struct EvBackToIdleRequested : sc::event<EvBackToIdleRequested> {};

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
struct EvResumeToActivate : sc::event<EvResumeToActivate> {};
struct EvResumeToBackToIdle : sc::event<EvResumeToBackToIdle> {};

// BACK_TO_IDLE 内部恢复路由事件
struct EvBackToIdleResumeToP2 : sc::event<EvBackToIdleResumeToP2> {};
struct EvBackToIdleResumeToP1 : sc::event<EvBackToIdleResumeToP1> {};
struct EvBackToIdleDone : sc::event<EvBackToIdleDone> {};

// ACTIVATE 内部恢复路由事件
struct EvActivateResumeToP1 : sc::event<EvActivateResumeToP1> {};
struct EvActivateResumeToP2 : sc::event<EvActivateResumeToP2> {};

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

// PICK 内部恢复路由事件
struct EvPickResumeToLPregrasp : sc::event<EvPickResumeToLPregrasp> {};
struct EvPickResumeToLPregraspP1 : sc::event<EvPickResumeToLPregraspP1> {};
struct EvPickResumeToLPregraspP2 : sc::event<EvPickResumeToLPregraspP2> {};
struct EvPickResumeToGripperOpen : sc::event<EvPickResumeToGripperOpen> {};
struct EvPickResumeToCartesianDown : sc::event<EvPickResumeToCartesianDown> {};
struct EvPickResumeToGripperClose : sc::event<EvPickResumeToGripperClose> {};
struct EvPickResumeToCartesianUp : sc::event<EvPickResumeToCartesianUp> {};
struct EvPickResumeToLRetreat : sc::event<EvPickResumeToLRetreat> {};

// PLACE 内部恢复路由事件
struct EvPlaceResumeToPre : sc::event<EvPlaceResumeToPre> {};
struct EvPlaceResumeToMove : sc::event<EvPlaceResumeToMove> {};
struct EvPlaceResumeToRelease : sc::event<EvPlaceResumeToRelease> {};
struct EvPlaceResumeToRetreat : sc::event<EvPlaceResumeToRetreat> {};
struct EvPlaceResumeToClose : sc::event<EvPlaceResumeToClose> {};
struct EvPlaceReleased : sc::event<EvPlaceReleased> {};

}  // namespace je_arm_pcb_inspection_sm
