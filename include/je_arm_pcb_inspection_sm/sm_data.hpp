#pragma once

#include <string>

namespace je_arm_pcb_inspection_sm
{
namespace sm_data
{
inline constexpr const char * kResumeStateId = "resume_state_id";
inline constexpr const char * kWaitResourcesState = "WAIT_RESOURCES";
inline constexpr const char * kWorkState = "WORK";
inline constexpr const char * kBackState = "BACK";

inline constexpr const char * kResumeFromPause = "resume_from_pause";
inline constexpr const char * kWorkResumeSubstateId = "work_resume_substate_id";
inline constexpr const char * kWorkSubstatePick = "WORK_PICK";
inline constexpr const char * kWorkSubstateInspect = "WORK_INSPECT";
inline constexpr const char * kWorkSubstateSelectBin = "WORK_SELECT_BIN";
inline constexpr const char * kWorkSubstatePlace = "WORK_PLACE";
inline constexpr const char * kWorkSubstatePlaceDecision = "WORK_PLACE_DECISION";

inline constexpr const char * kPickResumeFromPause = "pick_resume_from_pause";
inline constexpr const char * kPickResumeSubstateId = "pick_resume_substate_id";
inline constexpr const char * kPickSubstateLPregrasp = "PICK_L_PREGRASP";  // legacy: falls back to P1
inline constexpr const char * kPickSubstateLPregraspP1 = "PICK_L_PREGRASP_P1";
inline constexpr const char * kPickSubstateLPregraspP2 = "PICK_L_PREGRASP_P2";
inline constexpr const char * kPickSubstateLPregraspP3 = "PICK_L_PREGRASP_P3";
inline constexpr const char * kPickSubstateGripperOpen = "PICK_GRIPPER_OPEN";
inline constexpr const char * kPickSubstateCartesianDown = "PICK_CARTESIAN_DOWN";
inline constexpr const char * kPickSubstateGripperClose = "PICK_GRIPPER_CLOSE";
inline constexpr const char * kPickSubstateCartesianUp = "PICK_CARTESIAN_UP";
inline constexpr const char * kPickSubstateLRetreat = "PICK_L_RETREAT";

inline constexpr const char * kPcbPresent = "pcb_present";
inline constexpr const char * kLeftSlotFree = "left_slot_free";
inline constexpr const char * kRightSlotFree = "right_slot_free";

inline constexpr const char * kWaitTimeoutSec = "wait_timeout_sec";
inline constexpr const char * kWorkCycleSec = "work_cycle_sec";
inline constexpr const char * kBackHomeSec = "back_home_sec";
}  // namespace sm_data
}  // namespace je_arm_pcb_inspection_sm