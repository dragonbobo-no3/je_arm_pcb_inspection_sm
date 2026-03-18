#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;

// 前向声明
struct StBack;
struct StPauseResumeRouter;

/// PAUSE 状态：人工恢复
/// 故障、可恢复异常的集中处理，等待操作员手动指令
struct StPause : smacc2::SmaccState<StPause, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvFaultToBack, StBack>,
    smacc2::Transition<EvKeyBack, StBack>,
    smacc2::Transition<EvKeyResume, StPauseResumeRouter>
  > reactions;

  void onEntry() 
  { 
    std::string resumeState = sm_data::kWaitResourcesState;
    std::string workSubstate = sm_data::kWorkSubstatePick;
    std::string pickSubstate = sm_data::kPickSubstateLPregraspP1;
    std::string pauseReason;

    this->getGlobalSMData(std::string(sm_data::kResumeStateId), resumeState);
    this->getGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), workSubstate);
    this->getGlobalSMData(std::string(sm_data::kPickResumeSubstateId), pickSubstate);
    this->getGlobalSMData(std::string(sm_data::kPauseReason), pauseReason);

    if (pauseReason.empty())
    {
      pauseReason =
        std::string("[") + log_utils::bjtNowString() + "] 自动进入暂停: 可能为动作执行失败, work=" +
        workSubstate + ", pick=" + pickSubstate + ", resume=" + resumeState;
    }

    this->setGlobalSMData(std::string(sm_data::kPauseReason), pauseReason);

    RCLCPP_WARN(
      log_utils::bizLogger(),
      "[%s] ENTER PAUSE | resume=%s | reason=%s",
      log_utils::bjtNowString().c_str(),
      resumeState.c_str(),
      pauseReason.c_str());
  }

  void onExit() 
  { 
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT PAUSE", log_utils::bjtNowString().c_str());
  }
};

}  // namespace je_arm_pcb_inspection_sm
