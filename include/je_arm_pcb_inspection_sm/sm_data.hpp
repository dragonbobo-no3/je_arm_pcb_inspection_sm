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
inline constexpr const char * kBackToIdleState = "BACK_TO_IDLE";
inline constexpr const char * kBackToIdleResumeSubstateId = "back_to_idle_resume_substate_id";
inline constexpr const char * kBackToIdleResumeFromPause = "back_to_idle_resume_from_pause";
inline constexpr const char * kBackToIdleSubstateP2 = "BACK_TO_IDLE_P2";
inline constexpr const char * kBackToIdleSubstateP1 = "BACK_TO_IDLE_P1";
inline constexpr const char * kBackToIdleSubstateDone = "BACK_TO_IDLE_DONE";

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
inline constexpr const char * kWorkDelayNextSubstateId = "work_delay_next_substate_id";

inline constexpr const char * kPickResumeFromPause = "pick_resume_from_pause";
inline constexpr const char * kPickResumeSubstateId = "pick_resume_substate_id";
inline constexpr const char * kPickSubstateLPregrasp = "PICK_L_PREGRASP";
inline constexpr const char * kPickSubstateLPregraspP1 = "PICK_L_PREGRASP_P1";
inline constexpr const char * kPickSubstateLPregraspP2 = "PICK_L_PREGRASP_P2";
inline constexpr const char * kPickSubstateGripperOpen = "PICK_GRIPPER_OPEN";
inline constexpr const char * kPickSubstateCartesianDown = "PICK_CARTESIAN_DOWN";
inline constexpr const char * kPickSubstateGripperClose = "PICK_GRIPPER_CLOSE";
inline constexpr const char * kPickSubstateCartesianUp = "PICK_CARTESIAN_UP";
inline constexpr const char * kPickSubstateLRetreat = "PICK_L_RETREAT";
inline constexpr const char * kPickDelayNextSubstateId = "pick_delay_next_substate_id";

inline constexpr const char * kPlaceResumeFromPause = "place_resume_from_pause";
inline constexpr const char * kPlaceResumeSubstateId = "place_resume_substate_id";
inline constexpr const char * kPlaceSubstateLPregrasp = "PLACE_L_PREGRASP";
inline constexpr const char * kPlaceSubstateCartesianDown = "PLACE_CARTESIAN_DOWN";
inline constexpr const char * kPlaceSubstateGripperOpen = "PLACE_GRIPPER_OPEN";
inline constexpr const char * kPlaceSubstateCartesianUp = "PLACE_CARTESIAN_UP";
inline constexpr const char * kPlaceSubstateGripperClose = "PLACE_GRIPPER_CLOSE";
inline constexpr const char * kPlaceDelayNextSubstateId = "place_delay_next_substate_id";

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
inline constexpr const char * kPlaceTargetFrameId = "place_target_frame_id";
inline constexpr const char * kPlaceTargetX = "place_target_x";
inline constexpr const char * kPlaceTargetY = "place_target_y";
inline constexpr const char * kPlaceTargetZ = "place_target_z";
inline constexpr const char * kPlaceTargetQx = "place_target_qx";
inline constexpr const char * kPlaceTargetQy = "place_target_qy";
inline constexpr const char * kPlaceTargetQz = "place_target_qz";
inline constexpr const char * kPlaceTargetQw = "place_target_qw";
inline constexpr const char * kWaitResourcesPoseFrameId = "wait_resources_pose_frame_id";
inline constexpr const char * kWaitResourcesPoseX = "wait_resources_pose_x";
inline constexpr const char * kWaitResourcesPoseY = "wait_resources_pose_y";
inline constexpr const char * kWaitResourcesPoseZ = "wait_resources_pose_z";
inline constexpr const char * kWaitResourcesPoseQx = "wait_resources_pose_qx";
inline constexpr const char * kWaitResourcesPoseQy = "wait_resources_pose_qy";
inline constexpr const char * kWaitResourcesPoseQz = "wait_resources_pose_qz";
inline constexpr const char * kWaitResourcesPoseQw = "wait_resources_pose_qw";

inline constexpr const char * kWaitTimeoutSec = "wait_timeout_sec";
inline constexpr const char * kWorkCycleSec = "work_cycle_sec";
inline constexpr const char * kBackHomeSec = "back_home_sec";
inline constexpr const char * kSharedDelaySec = "shared_delay_sec";
}  // namespace sm_data
}  // namespace je_arm_pcb_inspection_sm