#pragma once

#include <string>

#include <smacc2/smacc.hpp>

#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

class CpTopLevelFlow : public smacc2::ISmaccComponent
{
public:
  CpTopLevelFlow() = default;
  virtual ~CpTopLevelFlow() = default;

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
  }

  void setResumeTarget(const char * target)
  {
    this->getStateMachine()->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(target));
  }

  bool isWorkReady()
  {
    const bool pcbPresent = getBoolData(sm_data::kPcbPresent, true);
    const bool leftSlotFree = getBoolData(sm_data::kLeftSlotFree, true);
    const bool rightSlotFree = getBoolData(sm_data::kRightSlotFree, true);
    return pcbPresent && (leftSlotFree || rightSlotFree);
  }

  void setPcbPresent(bool value)
  {
    this->getStateMachine()->setGlobalSMData(std::string(sm_data::kPcbPresent), value);
  }

  double waitTimeoutSeconds()
  {
    return getDoubleData(sm_data::kWaitTimeoutSec, 10.0);
  }

  double workCycleSeconds()
  {
    return getDoubleData(sm_data::kWorkCycleSec, 2.0);
  }

  double backHomeSeconds()
  {
    return getDoubleData(sm_data::kBackHomeSec, 1.5);
  }

private:
  bool getBoolData(const char * key, bool fallback)
  {
    bool value = fallback;
    this->getStateMachine()->getGlobalSMData(std::string(key), value);
    return value;
  }

  double getDoubleData(const char * key, double fallback)
  {
    double value = fallback;
    this->getStateMachine()->getGlobalSMData(std::string(key), value);
    return value;
  }
};

}  // namespace je_arm_pcb_inspection_sm