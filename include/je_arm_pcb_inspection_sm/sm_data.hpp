#pragma once

#include <string>

namespace je_arm_pcb_inspection_sm
{
namespace sm_data
{
inline constexpr const char * kResumeStateId = "resume_state_id";
inline constexpr const char * kActivateState = "ACTIVATE";
inline constexpr const char * kWaitResourcesState = "WAIT_RESOURCES";
inline constexpr const char * kWorkState = "WORK";
inline constexpr const char * kBackState = "BACK";

inline constexpr const char * kResumeFromPause = "resume_from_pause";
inline constexpr const char * kActivateResumeFromPause = "activate_resume_from_pause";
inline constexpr const char * kActivateResumeSubstateId = "activate_resume_substate_id";
inline constexpr const char * kActivateSubstateP1 = "ACTIVATE_P1";
inline constexpr const char * kActivateSubstateP2 = "ACTIVATE_P2";
inline constexpr const char * kActivateSubstateDone = "ACTIVATE_DONE";
inline constexpr const char * kActivateDelayNextSubstateId = "activate_delay_next_substate_id";
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
inline constexpr const char * kPickDelayNextSubstateId = "pick_delay_next_substate_id";

inline constexpr const char * kPcbPresent = "pcb_present";
inline constexpr const char * kLeftSlotFree = "left_slot_free";
inline constexpr const char * kRightSlotFree = "right_slot_free";
inline constexpr const char * kPauseReason = "pause_reason";
inline constexpr const char * kPcbTargetFrameId = "pcb_target_frame_id";
inline constexpr const char * kPcbTargetX = "pcb_target_x";
inline constexpr const char * kPcbTargetY = "pcb_target_y";
inline constexpr const char * kPcbTargetZ = "pcb_target_z";
inline constexpr const char * kPcbTargetQx = "pcb_target_qx";
inline constexpr const char * kPcbTargetQy = "pcb_target_qy";
inline constexpr const char * kPcbTargetQz = "pcb_target_qz";
inline constexpr const char * kPcbTargetQw = "pcb_target_qw";

inline constexpr const char * kWaitTimeoutSec = "wait_timeout_sec";
inline constexpr const char * kWorkCycleSec = "work_cycle_sec";
inline constexpr const char * kBackHomeSec = "back_home_sec";
inline constexpr const char * kSharedDelaySec = "shared_delay_sec";
}  // namespace sm_data
}  // namespace je_arm_pcb_inspection_sm