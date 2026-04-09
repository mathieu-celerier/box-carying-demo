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
  double nominalPositionStiffness = 100.0;
  double nominalPositionDamping = 40.0;
  double compliantPositionStiffness = 0.0;
  Eigen::Vector3d compliantPositionDamping = Eigen::Vector3d::Constant(10.0);
  double currentPositionStiffness = nominalPositionStiffness;
  Eigen::Vector3d currentPositionDamping = Eigen::Vector3d::Constant(nominalPositionDamping);
  Eigen::Vector3d nominalOrientationStiffness = Eigen::Vector3d::Constant(100.0);
  Eigen::Vector3d nominalOrientationDamping = Eigen::Vector3d::Constant(40.0);
  Eigen::Vector3d compliantOrientationStiffness = Eigen::Vector3d::Constant(10.0);
  Eigen::Vector3d compliantOrientationDamping = Eigen::Vector3d::Constant(40.0);
  Eigen::Vector3d xyForceCompliantOrientationStiffness = compliantOrientationStiffness;
  Eigen::Vector3d xyForceCompliantOrientationDamping = compliantOrientationDamping;
  Eigen::Vector3d rotationYCompliantOrientationStiffness = compliantOrientationStiffness;
  Eigen::Vector3d rotationYCompliantOrientationDamping = compliantOrientationDamping;
  Eigen::Vector3d currentOrientationStiffness = nominalOrientationStiffness;
  Eigen::Vector3d currentOrientationDamping = nominalOrientationDamping;
  double gainTransitionTime = 0.2;
  double xyComplianceOnThreshold = 10.0;
  double xyComplianceOffThreshold = 8;
  double dFZComplianceOnThreshold = 5.0;
  double dFZComplianceOffThreshold = 3.0;
  double rotationYComplianceOnThreshold = 10.0;
  double rotationYComplianceOffThreshold = 8.0;
  double rotationYComplianceYThreshold = 0.3;
  double complianceGain = 0.8;
  double residualGain = 10.0;
  bool refAccelFilterInitialized = false;
  bool xyForceTriggerActive = false;
  bool dFZComplianceActive = false;
  bool rotationYComplianceActive = false;
  bool updateGain = false;
};
