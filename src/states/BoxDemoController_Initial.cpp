#include "BoxDemoController_Initial.h"

#include "../BoxDemoController.h"

void BoxDemoController_Initial::configure(const mc_rtc::Configuration & config) {}

void BoxDemoController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BoxDemoController &>(ctl_);

  next = false;
  ctl.gui()->addElement({"Controller"}, mc_rtc::gui::Button("Start Demonstration", [this]() { next = true; }));
}

bool BoxDemoController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BoxDemoController &>(ctl_);
  if(next)
  {
    output("OK");
    return true;
  }
  else
  {
    return false;
  }
}

void BoxDemoController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BoxDemoController &>(ctl_);
}

EXPORT_SINGLE_STATE("BoxDemoController_Initial", BoxDemoController_Initial)
