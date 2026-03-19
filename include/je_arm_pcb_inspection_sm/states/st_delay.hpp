#pragma once

#include <chrono>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;
struct StActivate;
struct StWaitResources;
struct StWork;
struct StBack;
struct StPause;

struct StDelay : smacc2::SmaccState<StDelay, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvResumeToActivate, StActivate>,
    smacc2::Transition<EvResumeToWaitResources, StWaitResources>,
    smacc2::Transition<EvResumeToWork, StWork>,
    smacc2::Transition<EvResumeToBack, StBack>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    transitionPosted_ = false;

    this->getGlobalSMData(std::string(sm_data::kResumeStateId), resumeState_);
    this->getGlobalSMData(std::string(sm_data::kSharedDelaySec), delaySec_);

    if (resumeState_ == sm_data::kActivateState)
    {
      // Advance the Activate resume pointer to the next sub-state.
      std::string activateNext = sm_data::kActivateSubstateP1;
      this->getGlobalSMData(std::string(sm_data::kActivateDelayNextSubstateId), activateNext);
      this->setGlobalSMData(std::string(sm_data::kActivateResumeSubstateId), activateNext);
    }

    std::string workSubstate = sm_data::kWorkSubstatePick;
    this->getGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), workSubstate);
    if (resumeState_ == sm_data::kWorkState && workSubstate == sm_data::kWorkSubstatePick)
    {
      std::string pickNext = sm_data::kPickSubstateLPregraspP3;
      this->getGlobalSMData(std::string(sm_data::kPickDelayNextSubstateId), pickNext);
      this->setGlobalSMData(std::string(sm_data::kPickResumeSubstateId), pickNext);
    }

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER DELAY | sec=%.2f | resume=%s",
      log_utils::bjtNowString().c_str(),
      delaySec_,
      resumeState_.c_str());

    if (delaySec_ <= 0.0)
    {
      transitionPosted_ = true;
      dispatchResumeEvent();
      return;
    }

    auto node = this->getStateMachine().getNode();
    const auto delayDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(delaySec_));
    timer_ = node->create_wall_timer(
      delayDuration,
      [this]()
      {
        if (transitionPosted_)
        {
          return;
        }

        transitionPosted_ = true;
        if (timer_)
        {
          timer_->cancel();
        }
        dispatchResumeEvent();
      });
  }

  void onExit()
  {
    if (timer_)
    {
      timer_->cancel();
      timer_.reset();
    }
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT DELAY", log_utils::bjtNowString().c_str());
  }

private:
  void dispatchResumeEvent()
  {
    if (resumeState_ == sm_data::kActivateState)
    {
      this->template postEvent<EvResumeToActivate>();
    }
    else if (resumeState_ == sm_data::kWorkState)
    {
      this->setGlobalSMData(std::string(sm_data::kResumeFromPause), true);
      this->template postEvent<EvResumeToWork>();
    }
    else if (resumeState_ == sm_data::kBackState)
    {
      this->template postEvent<EvResumeToBack>();
    }
    else
    {
      this->template postEvent<EvResumeToWaitResources>();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::string resumeState_{sm_data::kWaitResourcesState};
  double delaySec_{0.6};
  bool transitionPosted_{false};
};

}  // namespace je_arm_pcb_inspection_sm
