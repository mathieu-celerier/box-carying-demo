#pragma once

#include <mc_control/fsm/State.h>

struct BoxDemoController;

struct BoxDemoController_Compliance : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  bool stop;
  rbd::Jacobian jac_ft;
  rbd::Jacobian jac_task;
  rbd::ForwardDynamics fd;

  Eigen::Vector6d FsSensorRaw;
  Eigen::Vector6d FsRaw;
  Eigen::Vector6d Fs;
  Eigen::VectorXd tau_s;
  Eigen::Vector6d As;
  Eigen::Vector6d AsRaw;

  Eigen::VectorXd tau_r;
  Eigen::VectorXd tau_r_aug;
  Eigen::Vector6d Fr;
  Eigen::Vector6d Fr_aug;
  Eigen::Vector6d Ar;
  Eigen::Vector6d Ar_aug;

  Eigen::VectorXd dF;
  Eigen::VectorXd dA;
  Eigen::Vector6d comp;
  Eigen::Vector6d refAccelRaw;
  Eigen::Vector6d refAccel;
  double nominalPositionStiffness = 30.0;
  double nominalPositionDamping = 20.0;
  double compliantPositionStiffness = 0.0;
  double compliantPositionDamping = 10.0;
  double residualGain = 10.0;
  bool refAccelFilterInitialized = false;
  bool xyForceTriggerActive = false;
  bool dFZComplianceActive = false;
  bool updateGain = false;
};
