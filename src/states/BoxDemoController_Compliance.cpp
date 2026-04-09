#include "BoxDemoController_Compliance.h"

#include "../BoxDemoController.h"

#include <algorithm>
#include <cmath>

namespace
{

} // namespace

void BoxDemoController_Compliance::configure(const mc_rtc::Configuration & config)
{
  config("residualGain", residualGain);
  config("xyComplianceOnThreshold", xyComplianceOnThreshold);
  config("xyComplianceOffThreshold", xyComplianceOffThreshold);
  config("dFZComplianceOnThreshold", dFZComplianceOnThreshold);
  config("dFZComplianceOffThreshold", dFZComplianceOffThreshold);
  config("rotationYComplianceOnThreshold", rotationYComplianceOnThreshold);
  config("rotationYComplianceOffThreshold", rotationYComplianceOffThreshold);
  config("rotationYComplianceXThreshold", rotationYComplianceYThreshold);
  config("complianceGain", complianceGain);
  config("nominalPositionStiffness", nominalPositionStiffness);
  config("nominalPositionDamping", nominalPositionDamping);
  config("compliantPositionStiffness", compliantPositionStiffness);
  config("compliantPositionDamping", compliantPositionDamping);
  config("nominalOrientationStiffness", nominalOrientationStiffness);
  config("nominalOrientationDamping", nominalOrientationDamping);
  config("compliantOrientationStiffness", compliantOrientationStiffness);
  config("compliantOrientationDamping", compliantOrientationDamping);
  xyForceCompliantOrientationStiffness = compliantOrientationStiffness;
  xyForceCompliantOrientationDamping = compliantOrientationDamping;
  rotationYCompliantOrientationStiffness = compliantOrientationStiffness;
  rotationYCompliantOrientationDamping = compliantOrientationDamping;
  config("xyForceCompliantOrientationStiffness", xyForceCompliantOrientationStiffness);
  config("xyForceCompliantOrientationDamping", xyForceCompliantOrientationDamping);
  config("rotationYCompliantOrientationStiffness", rotationYCompliantOrientationStiffness);
  config("rotationYCompliantOrientationDamping", rotationYCompliantOrientationDamping);
  config("gainTransitionTime", gainTransitionTime);

  residualGain = std::max(0.1, residualGain);
  xyComplianceOnThreshold = std::max(0.0, xyComplianceOnThreshold);
  xyComplianceOffThreshold = std::max(0.0, std::min(xyComplianceOffThreshold, xyComplianceOnThreshold));
  dFZComplianceOnThreshold = std::max(0.0, dFZComplianceOnThreshold);
  dFZComplianceOffThreshold = std::max(0.0, std::min(dFZComplianceOffThreshold, dFZComplianceOnThreshold));
  rotationYComplianceOnThreshold = std::max(0.0, rotationYComplianceOnThreshold);
  rotationYComplianceOffThreshold =
      std::max(0.0, std::min(rotationYComplianceOffThreshold, rotationYComplianceOnThreshold));
  rotationYComplianceYThreshold = std::max(0.0, rotationYComplianceYThreshold);
  complianceGain = std::max(0.0, complianceGain);
  nominalPositionStiffness = std::max(0.0, nominalPositionStiffness);
  nominalPositionDamping = std::max(0.0, nominalPositionDamping);
  compliantPositionStiffness = std::max(0.0, compliantPositionStiffness);
  compliantPositionDamping = compliantPositionDamping.cwiseMax(0.0);
  nominalOrientationStiffness = nominalOrientationStiffness.cwiseMax(0.0);
  nominalOrientationDamping = nominalOrientationDamping.cwiseMax(0.0);
  compliantOrientationStiffness = compliantOrientationStiffness.cwiseMax(0.0);
  compliantOrientationDamping = compliantOrientationDamping.cwiseMax(0.0);
  xyForceCompliantOrientationStiffness = xyForceCompliantOrientationStiffness.cwiseMax(0.0);
  xyForceCompliantOrientationDamping = xyForceCompliantOrientationDamping.cwiseMax(0.0);
  rotationYCompliantOrientationStiffness = rotationYCompliantOrientationStiffness.cwiseMax(0.0);
  rotationYCompliantOrientationDamping = rotationYCompliantOrientationDamping.cwiseMax(0.0);
  gainTransitionTime = std::max(1e-3, gainTransitionTime);

  currentPositionStiffness = nominalPositionStiffness;
  currentPositionDamping = Eigen::Vector3d::Constant(nominalPositionDamping);
  currentOrientationStiffness = nominalOrientationStiffness;
  currentOrientationDamping = nominalOrientationDamping;
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
  rotationYComplianceActive = false;
  currentPositionStiffness = nominalPositionStiffness;
  currentPositionDamping = Eigen::Vector3d::Constant(nominalPositionDamping);
  currentOrientationStiffness = nominalOrientationStiffness;
  currentOrientationDamping = nominalOrientationDamping;
  ctl.gui()->addElement(
      {"Controller"}, mc_rtc::gui::Button("Stop Demonstration", [this]() { stop = true; }),
      mc_rtc::gui::NumberInput(
          "Residual gain", [this]() { return residualGain; },
          [this](double gain)
          {
            updateGain = true;
            residualGain = std::max(0.1, gain);
          }),
      mc_rtc::gui::NumberInput(
          "XY compliance on threshold", [this]() { return xyComplianceOnThreshold; },
          [this](double value) { xyComplianceOnThreshold = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "XY compliance off threshold", [this]() { return xyComplianceOffThreshold; },
          [this](double value) { xyComplianceOffThreshold = std::max(0.0, std::min(value, xyComplianceOnThreshold)); }),
      mc_rtc::gui::NumberInput(
          "dFz compliance on threshold", [this]() { return dFZComplianceOnThreshold; },
          [this](double value) { dFZComplianceOnThreshold = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "dFz compliance off threshold", [this]() { return dFZComplianceOffThreshold; }, [this](double value)
          { dFZComplianceOffThreshold = std::max(0.0, std::min(value, dFZComplianceOnThreshold)); }),
      mc_rtc::gui::NumberInput(
          "Rotation Y compliance on threshold", [this]() { return rotationYComplianceOnThreshold; },
          [this](double value) { rotationYComplianceOnThreshold = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Rotation Y compliance off threshold", [this]() { return rotationYComplianceOffThreshold; },
          [this](double value)
          { rotationYComplianceOffThreshold = std::max(0.0, std::min(value, rotationYComplianceOnThreshold)); }),
      mc_rtc::gui::NumberInput(
          "Rotation Y compliance X threshold", [this]() { return rotationYComplianceYThreshold; },
          [this](double value) { rotationYComplianceYThreshold = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Compliance gain", [this]() { return complianceGain; },
          [this](double value) { complianceGain = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Position nominal stiffness", [this]() { return nominalPositionStiffness; },
          [this](double value) { nominalPositionStiffness = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Position nominal damping", [this]() { return nominalPositionDamping; },
          [this](double value) { nominalPositionDamping = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Position compliant stiffness", [this]() { return compliantPositionStiffness; },
          [this](double value) { compliantPositionStiffness = std::max(0.0, value); }),
      mc_rtc::gui::ArrayInput(
          "Position compliant damping", {"x", "y", "z"}, [this]() { return compliantPositionDamping; },
          [this](const Eigen::Vector3d & value) { compliantPositionDamping = value.cwiseMax(0.0); }),
      mc_rtc::gui::ArrayInput(
          "Orientation nominal stiffness", {"x", "y", "z"}, [this]() { return nominalOrientationStiffness; },
          [this](const Eigen::Vector3d & value) { nominalOrientationStiffness = value.cwiseMax(0.0); }),
      mc_rtc::gui::ArrayInput(
          "Orientation nominal damping", {"x", "y", "z"}, [this]() { return nominalOrientationDamping; },
          [this](const Eigen::Vector3d & value) { nominalOrientationDamping = value.cwiseMax(0.0); }),
      mc_rtc::gui::ArrayInput(
          "Orientation compliant stiffness", {"x", "y", "z"}, [this]() { return compliantOrientationStiffness; },
          [this](const Eigen::Vector3d & value)
          {
            compliantOrientationStiffness = value.cwiseMax(0.0);
            xyForceCompliantOrientationStiffness = compliantOrientationStiffness;
            rotationYCompliantOrientationStiffness = compliantOrientationStiffness;
          }),
      mc_rtc::gui::ArrayInput(
          "Orientation compliant damping", {"x", "y", "z"}, [this]() { return compliantOrientationDamping; },
          [this](const Eigen::Vector3d & value)
          {
            compliantOrientationDamping = value.cwiseMax(0.0);
            xyForceCompliantOrientationDamping = compliantOrientationDamping;
            rotationYCompliantOrientationDamping = compliantOrientationDamping;
          }),
      mc_rtc::gui::ArrayInput(
          "XY force orientation stiffness", {"x", "y", "z"}, [this]() { return xyForceCompliantOrientationStiffness; },
          [this](const Eigen::Vector3d & value) { xyForceCompliantOrientationStiffness = value.cwiseMax(0.0); }),
      mc_rtc::gui::ArrayInput(
          "XY force orientation damping", {"x", "y", "z"}, [this]() { return xyForceCompliantOrientationDamping; },
          [this](const Eigen::Vector3d & value) { xyForceCompliantOrientationDamping = value.cwiseMax(0.0); }),
      mc_rtc::gui::ArrayInput(
          "Rotation Y orientation stiffness", {"x", "y", "z"},
          [this]() { return rotationYCompliantOrientationStiffness; },
          [this](const Eigen::Vector3d & value) { rotationYCompliantOrientationStiffness = value.cwiseMax(0.0); }),
      mc_rtc::gui::ArrayInput(
          "Rotation Y orientation damping", {"x", "y", "z"}, [this]() { return rotationYCompliantOrientationDamping; },
          [this](const Eigen::Vector3d & value) { rotationYCompliantOrientationDamping = value.cwiseMax(0.0); }),
      mc_rtc::gui::NumberInput(
          "Gain transition time", [this]() { return gainTransitionTime; },
          [this](double value) { gainTransitionTime = std::max(1e-3, value); }));

  jac_ft = rbd::Jacobian(robot.mb(), sensor_body);
  jac_task = rbd::Jacobian(robot.mb(), "tool");
  fd = rbd::ForwardDynamics(robot.mb());

  ctl.eeTask->positionTask->stiffness(nominalPositionStiffness);
  ctl.eeTask->positionTask->damping(nominalPositionDamping);
  ctl.eeTask->orientationTask->setGains(nominalOrientationStiffness, nominalOrientationDamping);
  ctl.postureTask->makeCompliant(true);

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
  auto & ctl = static_cast<BoxDemoController &>(ctl_);
  auto & robot = ctl.robot();
  if(updateGain)
  {
    updateGain = false;
    mc_rtc::log::info("Updating the residual gain");
    ctl.datastore().call<void, double>("EF_Estimator::setGain", residualGain);
  }

  auto R = ctl.robot().bodyPosW(robot.forceSensors()[0].parentBody()).rotation();
  Eigen::MatrixXd J_ft = jac_ft.jacobian(robot.mb(), robot.mbc());
  Eigen::MatrixXd J_task = jac_ft.jacobian(robot.mb(), robot.mbc());
  fd.computeH(robot.mb(), robot.mbc());
  const double dt = ctl.solver().dt();
  const double alpha = std::exp(-std::max(0.1, residualGain) * dt);
  const double gainAlpha = std::exp(-dt / std::max(1e-3, gainTransitionTime));

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
  refAccelRaw = complianceGain * Ar_aug;
  refAccelRaw.head(3).setZero();

  const bool hadSoftPositionGains = xyForceTriggerActive || dFZComplianceActive;
  const bool hadSoftOrientationGains = xyForceTriggerActive || rotationYComplianceActive;
  const double normDFxy = dF.tail(3).head(2).norm();
  const double absDTauY = std::abs(dF[1]);
  const double absDFz = std::abs(dF[5]);
  const double currentToolX = ctl.eeTask->orientationTask->frame_->position().translation().y();
  const bool allowRotationYCompliance = currentToolX > rotationYComplianceYThreshold;

  if(normDFxy > xyComplianceOnThreshold)
  {
    refAccelRaw[2] = Ar[2];
    refAccelRaw[3] = Ar[3];
    refAccelRaw[4] = Ar[4];
    xyForceTriggerActive = true;
  }
  else
  {
    if(normDFxy < xyComplianceOffThreshold)
    {
      refAccelRaw[2] = 0;
      refAccelRaw[3] = 0;
      refAccelRaw[4] = 0;
      xyForceTriggerActive = false;
    }
  }

  if(absDTauY > rotationYComplianceOnThreshold && allowRotationYCompliance)
  {
    refAccelRaw[1] = Ar[1];
    rotationYComplianceActive = true;
  }
  else if(absDTauY < rotationYComplianceOffThreshold || !allowRotationYCompliance)
  {
    refAccelRaw[1] = 0;
    if(absDTauY < rotationYComplianceOffThreshold)
    {
      rotationYComplianceActive = false;
    }
    if(!allowRotationYCompliance)
    {
      rotationYComplianceActive = false;
    }
  }

  if(absDFz > dFZComplianceOnThreshold)
  {
    refAccelRaw[5] = comp[5];
    dFZComplianceActive = true;
  }
  else
  {
    refAccelRaw[5] = 0;
    if(absDFz < dFZComplianceOffThreshold)
    {
      dFZComplianceActive = false;
    }
  }

  const bool hasSoftPositionGains = xyForceTriggerActive || dFZComplianceActive;
  const bool hasSoftOrientationGains = xyForceTriggerActive || rotationYComplianceActive;
  if(hadSoftPositionGains && !hasSoftPositionGains)
  {
    ctl.eeTask->positionTask->reset();
  }
  if(hadSoftOrientationGains && !hasSoftOrientationGains)
  {
    ctl.eeTask->orientationTask->reset();
  }
  const double targetPositionStiffness = hasSoftPositionGains ? compliantPositionStiffness : nominalPositionStiffness;
  const Eigen::Vector3d targetPositionDamping =
      hasSoftPositionGains ? compliantPositionDamping : Eigen::Vector3d::Constant(nominalPositionDamping);
  currentPositionStiffness = gainAlpha * currentPositionStiffness + (1.0 - gainAlpha) * targetPositionStiffness;
  currentPositionDamping = gainAlpha * currentPositionDamping + (1.0 - gainAlpha) * targetPositionDamping;
  ctl.eeTask->positionTask->setGains(Eigen::Vector3d::Constant(currentPositionStiffness), currentPositionDamping);
  Eigen::Vector3d targetOrientationStiffness = nominalOrientationStiffness;
  Eigen::Vector3d targetOrientationDamping = nominalOrientationDamping;
  if(xyForceTriggerActive)
  {
    targetOrientationStiffness.x() = xyForceCompliantOrientationStiffness.x();
    targetOrientationStiffness.z() = xyForceCompliantOrientationStiffness.z();
    targetOrientationDamping.x() = xyForceCompliantOrientationDamping.x();
    targetOrientationDamping.z() = xyForceCompliantOrientationDamping.z();
  }
  if(rotationYComplianceActive)
  {
    targetOrientationStiffness.y() = rotationYCompliantOrientationStiffness.y();
    targetOrientationDamping.y() = rotationYCompliantOrientationDamping.y();
  }
  currentOrientationStiffness =
      gainAlpha * currentOrientationStiffness + (1.0 - gainAlpha) * targetOrientationStiffness;
  currentOrientationDamping = gainAlpha * currentOrientationDamping + (1.0 - gainAlpha) * targetOrientationDamping;
  ctl.eeTask->orientationTask->setGains(currentOrientationStiffness, currentOrientationDamping);
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
