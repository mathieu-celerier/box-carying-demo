#include "BoxDemoController_Compliance.h"

#include "../BoxDemoController.h"

#include <algorithm>
#include <cmath>

void BoxDemoController_Compliance::configure(const mc_rtc::Configuration & config)
{
  residualGain = 10.0;
}

void BoxDemoController_Compliance::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BoxDemoController &>(ctl_);
  auto & robot = ctl.robot();
  auto & sensor_body = robot.forceSensors()[0].parentBody();
  const auto nrDof = static_cast<Eigen::Index>(robot.mb().nrDof());

  // Deactivate feedback from external forces estimator (safer)
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Activate force sensor usage if not used yet
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", residualGain);

  stop = false;
  FsSensorRaw.setZero();
  FsRaw.setZero();
  Fs.setZero();
  tau_s = Eigen::VectorXd::Zero(nrDof);
  As.setZero();
  AsRaw.setZero();
  tau_r = Eigen::VectorXd::Zero(nrDof);
  tau_r_aug = Eigen::VectorXd::Zero(nrDof);
  Fr.setZero();
  Fr_aug.setZero();
  Ar.setZero();
  Ar_aug.setZero();
  dF = Eigen::VectorXd::Zero(6);
  dA = Eigen::VectorXd::Zero(6);
  comp.setZero();
  refAccelRaw.setZero();
  refAccel.setZero();
  refAccelFilterInitialized = false;
  xyForceTriggerActive = false;
  dFZComplianceActive = false;
  ctl.gui()->addElement({"Controller"}, mc_rtc::gui::Button("Stop Demonstration", [this]() { stop = true; }),
                        mc_rtc::gui::NumberInput(
                            "Residual gain", [this]() { return residualGain; },
                            [this](double gain)
                            {
                              updateGain = true;
                              residualGain = std::max(0.1, gain);
                            }));

  jac_ft = rbd::Jacobian(robot.mb(), sensor_body);
  jac_task = rbd::Jacobian(robot.mb(), "tool");
  fd = rbd::ForwardDynamics(robot.mb());

  ctl.eeTask->positionTask->stiffness(nominalPositionStiffness);
  ctl.eeTask->positionTask->damping(nominalPositionDamping);
  ctl.eeTask->orientationTask->stiffness(30);
  ctl.eeTask->orientationTask->damping(20);
  ctl.postureTask->makeCompliant(false);

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  ctl.logger().addLogEntries(
      this, "BoxDemoCtrl_stop", [this]() { return stop; }, "BoxDemoCtrl_residualGain",
      [this]() { return residualGain; }, "BoxDemoCtrl_FsSensorRaw", [this]() { return FsSensorRaw; },
      "BoxDemoCtrl_FsRaw", [this]() { return FsRaw; }, "BoxDemoCtrl_Fs", [this]() { return Fs; }, "BoxDemoCtrl_tau_s",
      [this]() { return tau_s; }, "BoxDemoCtrl_AsRaw", [this]() { return AsRaw; }, "BoxDemoCtrl_As",
      [this]() { return As; }, "BoxDemoCtrl_tau_r", [this]() { return tau_r; }, "BoxDemoCtrl_tau_r_aug",
      [this]() { return tau_r_aug; }, "BoxDemoCtrl_Fr", [this]() { return Fr; }, "BoxDemoCtrl_Fr_aug",
      [this]() { return Fr_aug; }, "BoxDemoCtrl_Ar", [this]() { return Ar; }, "BoxDemoCtrl_Ar_aug",
      [this]() { return Ar_aug; }, "BoxDemoCtrl_dF", [this]() { return dF; }, "BoxDemoCtrl_dA", [this]() { return dA; },
      "BoxDemoCtrl_comp", [this]() { return comp; }, "BoxDemoCtrl_refAccelRaw", [this]() { return refAccelRaw; },
      "BoxDemoCtrl_refAccel", [this]() { return refAccel; });
}

bool BoxDemoController_Compliance::run(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("========== new step ==========");
  auto & ctl = static_cast<BoxDemoController &>(ctl_);
  auto & robot = ctl.robot();
  if(updateGain)
  {
    updateGain = false;
    mc_rtc::log::info("Updating the residual gain");
    ctl.datastore().call<void, double>("EF_Estimator::setGain", residualGain);
  }

  mc_rtc::log::info("robot state & = {}", fmt::ptr(&robot));

  auto R = ctl.robot().bodyPosW(robot.forceSensors()[0].parentBody()).rotation();
  Eigen::MatrixXd J_ft = jac_ft.jacobian(robot.mb(), robot.mbc());
  Eigen::MatrixXd J_task = jac_ft.jacobian(robot.mb(), robot.mbc());
  fd.computeH(robot.mb(), robot.mbc());
  const double dt = ctl.solver().dt();
  const double alpha = std::exp(-std::max(0.1, residualGain) * dt);

  // Dual compliance logic implementation
  FsSensorRaw = robot.forceSensors()[0].wrenchWithoutGravity(robot).vector();
  FsRaw = FsSensorRaw;
  FsRaw.head(3) = R.transpose() * FsRaw.head(3);
  FsRaw.tail(3) = R.transpose() * FsRaw.tail(3);
  if(!refAccelFilterInitialized)
  {
    Fs = FsRaw;
    refAccelFilterInitialized = true;
  }
  else
  {
    Fs = alpha * Fs + (1.0 - alpha) * FsRaw;
  }
  tau_s = J_ft.transpose() * Fs;
  As = J_task * fd.H().inverse() * J_ft.transpose() * Fs;
  AsRaw = J_task * fd.H().inverse() * J_ft.transpose() * FsRaw;

  // tau_r = robot.externalTorques();
  tau_r = ctl.datastore().get<Eigen::VectorXd>("EF_Estimator::getResidualOnly");
  tau_r_aug = robot.externalTorques();
  Fr = J_ft.transpose().completeOrthogonalDecomposition().solve(tau_r);
  Fr_aug = J_ft.transpose().completeOrthogonalDecomposition().solve(tau_r_aug);
  Ar = J_task * fd.H().ldlt().solve(tau_r);
  Ar_aug = J_task * fd.H().ldlt().solve(tau_r_aug);

  dF = Fr - Fs;
  dA = Ar - As;
  comp = dA;
  refAccelRaw = 1.0 * Ar_aug;
  refAccelRaw.head(3).setZero();

  const bool hadSoftPositionGains = xyForceTriggerActive || dFZComplianceActive;
  const double normFxy = Fr.tail(3).head(2).norm();
  const double absDFz = std::abs(dF[5]);

  if(normFxy > 10)
  {
    xyForceTriggerActive = true;
  }
  else
  {
    if(normFxy < 1.5)
    {
      xyForceTriggerActive = false;
    }
  }

  if(absDFz > 10)
  {
    refAccelRaw[5] = comp[5];
    dFZComplianceActive = true;
  }
  else
  {
    refAccelRaw[5] = 0;
    if(absDFz < 1.5)
    {
      dFZComplianceActive = false;
    }
  }

  const bool hasSoftPositionGains = xyForceTriggerActive || dFZComplianceActive;
  if(hadSoftPositionGains && !hasSoftPositionGains)
  {
    ctl.eeTask->positionTask->reset();
  }
  if(hasSoftPositionGains)
  {
    ctl.eeTask->positionTask->stiffness(compliantPositionStiffness);
    ctl.eeTask->positionTask->damping(compliantPositionDamping);
  }
  else
  {
    ctl.eeTask->positionTask->stiffness(nominalPositionStiffness);
    ctl.eeTask->positionTask->damping(nominalPositionDamping);
  }
  ctl.eeTask->refAccel(refAccelRaw);

  // State output management
  if(stop)
  {
    output("OK");
    return true;
  }
  else
  {
    return false;
  }
}

void BoxDemoController_Compliance::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BoxDemoController &>(ctl_);
  ctl.logger().removeLogEntries(this);
}

EXPORT_SINGLE_STATE("BoxDemoController_Compliance", BoxDemoController_Compliance)
