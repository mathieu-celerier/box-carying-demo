#include "BoxDemoController_Compliance.h"

#include <mc_control/fsm/Controller.h>

#include "../BoxDemoController_WorkspaceConstraint.h"

#include <mc_rtc/gui.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/visual_utils.h>
#include <mc_tvm/VectorOrientationFunction.h>
#include <algorithm>
#include <cmath>

namespace
{

Eigen::Vector3d projectOntoPlane(const Eigen::Vector3d & vector, const Eigen::Vector3d & normal)
{
  return vector - vector.dot(normal) * normal;
}

Eigen::Vector3d normalizedOrFallback(const Eigen::Vector3d & vector, const Eigen::Vector3d & fallback)
{
  if(vector.squaredNorm() > 1e-12)
  {
    return vector.normalized();
  }
  return fallback.normalized();
}

Eigen::Matrix3d projectOrientationKeepingYAxisUp(const Eigen::Matrix3d & rotation)
{
  const Eigen::Vector3d worldZ = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d xProjected = projectOntoPlane(rotation.col(0), worldZ);
  xProjected = normalizedOrFallback(xProjected, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d yProjected = worldZ;
  const Eigen::Vector3d zProjected = xProjected.cross(yProjected).normalized();

  Eigen::Matrix3d projected;
  projected.col(0) = xProjected;
  projected.col(1) = yProjected;
  projected.col(2) = zProjected;
  return projected;
}

} // namespace

void BoxDemoController_Compliance_AdjustableVectorOrientationTask::frameVector(const Eigen::Vector3d & vector)
{
  const Eigen::Vector3d normalized = vector.normalized();
  switch(backend())
  {
    case Backend::Tasks:
      static_cast<tasks::qp::VectorOrientationTask *>(errorT.get())
          ->bodyVector((sva::PTransformd{normalized} * frame_->X_b_f()).translation().normalized());
      break;
    case Backend::TVM:
      static_cast<mc_tvm::VectorOrientationFunction *>(errorT.get())->frameVector(normalized);
      break;
    default:
      mc_rtc::log::error_and_throw(
          "[BoxDemoController_Compliance] Unsupported backend for adjustable VectorOrientationTask");
  }
}

std::string BoxDemoController_Compliance::orientationManagementModeName(OrientationManagementMode mode)
{
  switch(mode)
  {
    case OrientationManagementMode::Compliance:
      return "Compliance";
    case OrientationManagementMode::WorldZTorqueToLocalY:
      return "Admittance";
  }
  return "Compliance";
}

BoxDemoController_Compliance::OrientationManagementMode BoxDemoController_Compliance::orientationManagementModeFromName(
    const std::string & mode)
{
  if(mode == "Admittance")
  {
    return OrientationManagementMode::WorldZTorqueToLocalY;
  }
  return OrientationManagementMode::Compliance;
}

void BoxDemoController_Compliance::updateSlidingAverage(std::deque<double> & history,
                                                        double & sum,
                                                        double value,
                                                        size_t maxSamples)
{
  history.push_back(value);
  sum += value;
  while(history.size() > maxSamples)
  {
    sum -= history.front();
    history.pop_front();
  }
}

void BoxDemoController_Compliance::configure(const mc_rtc::Configuration & config)
{
  config("postureHome", postureHome_);
  config("toolFrame", toolFrameName_);
  config("targetPosition", targetPosition_);
  config("targetOrientationQuaternion", targetOrientationQuaternion_);
  config("workspaceConstraintEnabled", workspaceConstraintEnabled_);
  config("workspaceMin", workspaceMin_);
  config("workspaceMax", workspaceMax_);
  config("residualGain", residualGain);
  config("planarTranslationTriggerOnThreshold", planarTranslationTriggerOnThreshold);
  config("planarTranslationTriggerOffThreshold", planarTranslationTriggerOffThreshold);
  config("rotationZTriggerOnThreshold", rotationZTriggerOnThreshold);
  config("rotationZTriggerOffThreshold", rotationZTriggerOffThreshold);
  config("dFZComplianceOnThreshold", dFZComplianceOnThreshold);
  config("dFZComplianceOffThreshold", dFZComplianceOffThreshold);
  config("triggerAveragingWindow", triggerAveragingWindow);
  config("complianceGain", complianceGain);
  config("nominalPositionStiffness", nominalPositionStiffness);
  config("nominalPositionDamping", nominalPositionDamping);
  config("planarTranslationStiffness", planarTranslationStiffness);
  config("planarTranslationDamping", planarTranslationDamping);
  config("nominalOrientationStiffness", nominalOrientationStiffness);
  config("nominalOrientationDamping", nominalOrientationDamping);
  config("rotationZOrientationStiffness", rotationZOrientationStiffness);
  config("rotationZOrientationDamping", rotationZOrientationDamping);
  config("worldZTorqueToLocalYGain", worldZTorqueToLocalYGain);
  config("maxLocalYOrientationOffset", maxLocalYOrientationOffset);
  config("exclusiveComplianceModes", exclusiveComplianceModes);
  config("gainTransitionTime", gainTransitionTime);
  std::string orientationMode = orientationManagementModeName(orientationManagementMode);
  config("orientationManagementMode", orientationMode);
  orientationManagementMode = orientationManagementModeFromName(orientationMode);

  residualGain = std::max(0.1, residualGain);
  planarTranslationTriggerOnThreshold = std::max(0.0, planarTranslationTriggerOnThreshold);
  planarTranslationTriggerOffThreshold =
      std::max(0.0, std::min(planarTranslationTriggerOffThreshold, planarTranslationTriggerOnThreshold));
  rotationZTriggerOnThreshold = std::max(0.0, rotationZTriggerOnThreshold);
  rotationZTriggerOffThreshold = std::max(0.0, std::min(rotationZTriggerOffThreshold, rotationZTriggerOnThreshold));
  dFZComplianceOnThreshold = std::max(0.0, dFZComplianceOnThreshold);
  dFZComplianceOffThreshold = std::max(0.0, std::min(dFZComplianceOffThreshold, dFZComplianceOnThreshold));
  triggerAveragingWindow = std::max(0.0, triggerAveragingWindow);
  complianceGain = std::max(0.0, complianceGain);
  nominalPositionStiffness = std::max(0.0, nominalPositionStiffness);
  nominalPositionDamping = std::max(0.0, nominalPositionDamping);
  planarTranslationStiffness = std::max(0.0, planarTranslationStiffness);
  planarTranslationDamping = planarTranslationDamping.cwiseMax(0.0);
  nominalOrientationStiffness = nominalOrientationStiffness.cwiseMax(0.0);
  nominalOrientationDamping = nominalOrientationDamping.cwiseMax(0.0);
  rotationZOrientationStiffness = std::max(0.0, rotationZOrientationStiffness);
  rotationZOrientationDamping = std::max(0.0, rotationZOrientationDamping);
  worldZTorqueToLocalYGain = std::max(0.0, worldZTorqueToLocalYGain);
  maxLocalYOrientationOffset = std::max(0.0, maxLocalYOrientationOffset);
  gainTransitionTime = std::max(1e-3, gainTransitionTime);

  currentPositionStiffness = nominalPositionStiffness;
  currentPositionDamping = Eigen::Vector3d::Constant(nominalPositionDamping);
  currentOrientationStiffness = nominalOrientationStiffness;
  currentOrientationDamping = nominalOrientationDamping;
  localYOrientationAngle = 0.0;
}

void BoxDemoController_Compliance::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = ctl_;
  auto & robot = ctl.robot();
  auto & sensor_body = robot.forceSensors()[0].parentBody();
  const auto nrDof = static_cast<Eigen::Index>(robot.mb().nrDof());
  auto defaultPostureTask = ctl.getPostureTask(robot.name());
  const Eigen::Matrix3d nominalToolOrientation = targetOrientation();

  // Deactivate feedback from external forces estimator (safer)
  estimatorWasActive_ = ctl.datastore().call<bool>("EF_Estimator::isActive");
  if(!estimatorWasActive_)
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Activate force sensor usage if not used yet
  estimatorWasUsingForceSensor_ = ctl.datastore().call<bool>("EF_Estimator::useForceSensor");
  if(!estimatorWasUsingForceSensor_)
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
  planarTranslationTriggerActive = false;
  rotationZTriggerActive = false;
  dFZComplianceActive = false;
  planarTranslationTriggerHistory.clear();
  rotationZTriggerHistory.clear();
  dFZComplianceHistory.clear();
  planarTranslationTriggerSum = 0.0;
  rotationZTriggerSum = 0.0;
  dFZComplianceSum = 0.0;
  currentPositionStiffness = nominalPositionStiffness;
  currentPositionDamping = Eigen::Vector3d::Constant(nominalPositionDamping);
  currentOrientationStiffness = nominalOrientationStiffness;
  currentOrientationDamping = nominalOrientationDamping;
  localYOrientationAngle = 0.0;
  const Eigen::Vector3d configuredWorkspaceMin = workspaceMin_;
  const Eigen::Vector3d configuredWorkspaceMax = workspaceMax_;
  workspaceMin_ = configuredWorkspaceMin.cwiseMin(configuredWorkspaceMax);
  workspaceMax_ = configuredWorkspaceMin.cwiseMax(configuredWorkspaceMax);

  eeTask_ =
      std::make_shared<mc_tasks::CompliantEndEffectorTask>(toolFrameName_, ctl.robots(), robot.robotIndex(), 2, 10000);
  eeTask_->orientationTask->orientation(nominalToolOrientation);
  eeTask_->positionTask->position(targetPosition_);
  ctl.solver().addTask(eeTask_);

  workspaceConstraint_ = std::make_shared<BoxDemoController_WorkspaceConstraint>(robot.frame(toolFrameName_),
                                                                                 workspaceMin_, workspaceMax_);
  if(workspaceConstraintEnabled_)
  {
    ctl.solver().addConstraintSet(*workspaceConstraint_);
  }

  nominalUpVectorInTool_ = nominalToolOrientation * Eigen::Vector3d::UnitZ();
  upVectorTask_ = std::make_shared<BoxDemoController_Compliance_AdjustableVectorOrientationTask>(
      robot.frame(toolFrameName_), nominalUpVectorInTool_, 2.0, 20000.0);
  upVectorTask_->targetVector(Eigen::Vector3d::UnitZ());
  ctl.solver().addTask(upVectorTask_);

  ctl.solver().removeTask(defaultPostureTask);
  postureTask_ = std::make_shared<mc_tasks::CompliantPostureTask>(ctl.solver(), robot.robotIndex(), 1, 1);
  postureTask_->reset();
  postureTask_->stiffness(1.0);
  postureTask_->damping(4.0);
  postureTask_->target(postureHome_);
  postureTask_->makeCompliant(true);
  ctl.solver().addTask(postureTask_);

  baseOrientationTarget = nominalToolOrientation;
  upVectorTask_->frameVector(nominalUpVectorInTool_);
  eeTask_->orientationTask->orientation(baseOrientationTarget);
  ctl.gui()->addElement(
      this, {"Controller", "Parameters"}, mc_rtc::gui::Button("Stop Demonstration", [this]() { stop = true; }),
      mc_rtc::gui::NumberInput(
          "Residual gain", [this]() { return residualGain; },
          [this](double gain)
          {
            updateGain = true;
            residualGain = std::max(0.1, gain);
          }),
      mc_rtc::gui::NumberInput(
          "Planar translation trigger on threshold", [this]() { return planarTranslationTriggerOnThreshold; },
          [this](double value) { planarTranslationTriggerOnThreshold = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Planar translation trigger off threshold", [this]() { return planarTranslationTriggerOffThreshold; },
          [this](double value)
          {
            planarTranslationTriggerOffThreshold = std::max(0.0, std::min(value, planarTranslationTriggerOnThreshold));
          }),
      mc_rtc::gui::NumberInput(
          "Rotation Z trigger on threshold", [this]() { return rotationZTriggerOnThreshold; },
          [this](double value) { rotationZTriggerOnThreshold = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Rotation Z trigger off threshold", [this]() { return rotationZTriggerOffThreshold; }, [this](double value)
          { rotationZTriggerOffThreshold = std::max(0.0, std::min(value, rotationZTriggerOnThreshold)); }),
      mc_rtc::gui::NumberInput(
          "dFz compliance on threshold", [this]() { return dFZComplianceOnThreshold; },
          [this](double value) { dFZComplianceOnThreshold = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "dFz compliance off threshold", [this]() { return dFZComplianceOffThreshold; }, [this](double value)
          { dFZComplianceOffThreshold = std::max(0.0, std::min(value, dFZComplianceOnThreshold)); }),
      mc_rtc::gui::NumberInput(
          "Trigger averaging window", [this]() { return triggerAveragingWindow; },
          [this](double value) { triggerAveragingWindow = std::max(0.0, value); }),
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
          "Planar translation stiffness", [this]() { return planarTranslationStiffness; },
          [this](double value) { planarTranslationStiffness = std::max(0.0, value); }),
      mc_rtc::gui::ArrayInput(
          "Planar translation damping", {"x", "y", "z"}, [this]() { return planarTranslationDamping; },
          [this](const Eigen::Vector3d & value) { planarTranslationDamping = value.cwiseMax(0.0); }),
      mc_rtc::gui::ArrayInput(
          "Orientation nominal stiffness", {"x", "y", "z"}, [this]() { return nominalOrientationStiffness; },
          [this](const Eigen::Vector3d & value) { nominalOrientationStiffness = value.cwiseMax(0.0); }),
      mc_rtc::gui::ArrayInput(
          "Orientation nominal damping", {"x", "y", "z"}, [this]() { return nominalOrientationDamping; },
          [this](const Eigen::Vector3d & value) { nominalOrientationDamping = value.cwiseMax(0.0); }),
      mc_rtc::gui::NumberInput(
          "Rotation Z orientation stiffness", [this]() { return rotationZOrientationStiffness; },
          [this](double value) { rotationZOrientationStiffness = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Rotation Z orientation damping", [this]() { return rotationZOrientationDamping; },
          [this](double value) { rotationZOrientationDamping = std::max(0.0, value); }),
      mc_rtc::gui::Checkbox(
          "Exclusive position/orientation compliance", [this]() { return exclusiveComplianceModes; },
          [this]() { exclusiveComplianceModes = !exclusiveComplianceModes; }),
      mc_rtc::gui::ComboInput(
          "Orientation management",
          {orientationManagementModeName(OrientationManagementMode::Compliance),
           orientationManagementModeName(OrientationManagementMode::WorldZTorqueToLocalY)},
          [this]() { return orientationManagementModeName(orientationManagementMode); },
          [this](const std::string & value) { orientationManagementMode = orientationManagementModeFromName(value); }),
      mc_rtc::gui::NumberInput(
          "World Z torque to local Y gain", [this]() { return worldZTorqueToLocalYGain; },
          [this](double value) { worldZTorqueToLocalYGain = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Max local Y orientation offset", [this]() { return maxLocalYOrientationOffset; },
          [this](double value) { maxLocalYOrientationOffset = std::max(0.0, value); }),
      mc_rtc::gui::NumberInput(
          "Gain transition time", [this]() { return gainTransitionTime; },
          [this](double value) { gainTransitionTime = std::max(1e-3, value); }));
  setupWorkspaceGUI(ctl);

  jac_ft = rbd::Jacobian(robot.mb(), sensor_body);
  jac_task = rbd::Jacobian(robot.mb(), toolFrameName_);
  fd = rbd::ForwardDynamics(robot.mb());

  eeTask_->positionTask->stiffness(nominalPositionStiffness);
  eeTask_->positionTask->damping(nominalPositionDamping);
  eeTask_->orientationTask->setGains(nominalOrientationStiffness, nominalOrientationDamping);
  eeTask_->setComplianceVector({0.0, 0.0, 0.0, 0.1, 0.1, 0.0});
  upVectorTask_->stiffness(100);
  postureTask_->makeCompliant(true);
  postureTask_->stiffness(10);

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  addComplianceLogs(ctl);
}

bool BoxDemoController_Compliance::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = ctl_;
  auto & robot = ctl.robot();
  if(updateGain)
  {
    updateGain = false;
    mc_rtc::log::info("Updating the residual gain");
    ctl.datastore().call<void, double>("EF_Estimator::setGain", residualGain);
  }

  auto R = ctl.robot().bodyPosW(robot.forceSensors()[0].parentBody()).rotation();
  Eigen::MatrixXd J_ft = jac_ft.jacobian(robot.mb(), robot.mbc());
  Eigen::MatrixXd J_task = jac_task.jacobian(robot.mb(), robot.mbc());
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

  const bool useComplianceOrientation = orientationManagementMode == OrientationManagementMode::Compliance;
  const bool hadSoftPositionGains = planarTranslationTriggerActive || dFZComplianceActive;
  const bool hadSoftOrientationGains = useComplianceOrientation && rotationZTriggerActive;
  const double normDFxy = dF.tail(3).head(2).norm();
  const double absDTauZ = std::abs(dF[2]);
  const double absDFz = std::abs(dF[5]);
  const size_t triggerAverageSamples =
      std::max<size_t>(1, static_cast<size_t>(std::ceil(std::max(0.0, triggerAveragingWindow) / dt)));

  updateSlidingAverage(planarTranslationTriggerHistory, planarTranslationTriggerSum, normDFxy, triggerAverageSamples);
  updateSlidingAverage(rotationZTriggerHistory, rotationZTriggerSum, absDTauZ, triggerAverageSamples);
  updateSlidingAverage(dFZComplianceHistory, dFZComplianceSum, absDFz, triggerAverageSamples);

  const double avgNormDFxy = planarTranslationTriggerSum / static_cast<double>(planarTranslationTriggerHistory.size());
  const double avgAbsDTauZ = rotationZTriggerSum / static_cast<double>(rotationZTriggerHistory.size());
  const double avgAbsDFz = dFZComplianceSum / static_cast<double>(dFZComplianceHistory.size());

  bool requestedPlanarTranslationTriggerActive = planarTranslationTriggerActive;
  if(avgNormDFxy > planarTranslationTriggerOnThreshold)
  {
    requestedPlanarTranslationTriggerActive = true;
  }
  else if(avgNormDFxy < planarTranslationTriggerOffThreshold)
  {
    requestedPlanarTranslationTriggerActive = false;
  }

  bool requestedRotationZTriggerActive = rotationZTriggerActive;
  if(avgAbsDTauZ > rotationZTriggerOnThreshold)
  {
    requestedRotationZTriggerActive = true;
  }
  else if(avgAbsDTauZ < rotationZTriggerOffThreshold)
  {
    requestedRotationZTriggerActive = false;
  }

  bool requestedDFZComplianceActive = dFZComplianceActive;
  if(avgAbsDFz > dFZComplianceOnThreshold)
  {
    requestedDFZComplianceActive = true;
  }
  else if(avgAbsDFz < dFZComplianceOffThreshold)
  {
    requestedDFZComplianceActive = false;
  }

  if(exclusiveComplianceModes)
  {
    const bool requestedOrientationCompliance = requestedRotationZTriggerActive;
    const bool requestedPositionCompliance = requestedPlanarTranslationTriggerActive || requestedDFZComplianceActive;
    if(requestedOrientationCompliance)
    {
      requestedPlanarTranslationTriggerActive = false;
      requestedDFZComplianceActive = false;
    }
    else if(requestedPositionCompliance)
    {
      requestedRotationZTriggerActive = false;
    }
  }

  planarTranslationTriggerActive = requestedPlanarTranslationTriggerActive;
  rotationZTriggerActive = requestedRotationZTriggerActive;
  dFZComplianceActive = requestedDFZComplianceActive;

  refAccelRaw[2] = useComplianceOrientation && rotationZTriggerActive ? Ar[2] : 0;
  refAccelRaw[3] = planarTranslationTriggerActive ? Ar[3] : 0;
  refAccelRaw[4] = planarTranslationTriggerActive ? Ar[4] : 0;
  refAccelRaw[5] = dFZComplianceActive ? comp[5] : 0;

  const bool hasSoftPositionGains = planarTranslationTriggerActive || dFZComplianceActive;
  const bool hasSoftOrientationGains = useComplianceOrientation && rotationZTriggerActive;
  if(hadSoftPositionGains && !hasSoftPositionGains)
  {
    eeTask_->positionTask->reset();
  }
  if(hadSoftOrientationGains && !hasSoftOrientationGains)
  {
    baseOrientationTarget =
        projectOrientationKeepingYAxisUp(eeTask_->orientationTask->frame_->position().rotation().transpose())
            .transpose();
  }
  const Eigen::Vector3d targetPositionDamping =
      hasSoftPositionGains ? planarTranslationDamping : Eigen::Vector3d::Constant(nominalPositionDamping);
  const double targetPositionStiffness = hasSoftPositionGains ? planarTranslationStiffness : nominalPositionStiffness;
  currentPositionStiffness = gainAlpha * currentPositionStiffness + (1.0 - gainAlpha) * targetPositionStiffness;
  currentPositionDamping = gainAlpha * currentPositionDamping + (1.0 - gainAlpha) * targetPositionDamping;
  eeTask_->positionTask->setGains(Eigen::Vector3d::Constant(currentPositionStiffness), currentPositionDamping);
  Eigen::Vector3d targetOrientationStiffness = nominalOrientationStiffness;
  Eigen::Vector3d targetOrientationDamping = nominalOrientationDamping;
  Eigen::Matrix3d targetOrientation = baseOrientationTarget;
  if(useComplianceOrientation && rotationZTriggerActive)
  {
    targetOrientationStiffness.z() = rotationZOrientationStiffness;
    targetOrientationDamping.z() = rotationZOrientationDamping;
  }
  else if(!useComplianceOrientation)
  {
    if(rotationZTriggerActive)
    {
      localYOrientationAngle = std::clamp(localYOrientationAngle - worldZTorqueToLocalYGain * dF[2] * dt,
                                          -maxLocalYOrientationOffset, maxLocalYOrientationOffset);
    }
    targetOrientation =
        baseOrientationTarget * Eigen::AngleAxisd(localYOrientationAngle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }
  else
  {
    localYOrientationAngle = 0.0;
  }
  currentOrientationStiffness =
      gainAlpha * currentOrientationStiffness + (1.0 - gainAlpha) * targetOrientationStiffness;
  currentOrientationDamping = gainAlpha * currentOrientationDamping + (1.0 - gainAlpha) * targetOrientationDamping;
  eeTask_->orientationTask->orientation(targetOrientation);
  eeTask_->orientationTask->setGains(currentOrientationStiffness, currentOrientationDamping);
  eeTask_->refAccel(refAccelRaw);

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
  auto & ctl = ctl_;
  ctl.logger().removeLogEntries(this);
  ctl.gui()->removeElements(this);
  ctl.datastore().assign<std::string>("ControlMode", "Position");

  if(workspaceConstraint_ && workspaceConstraint_->inSolver())
  {
    ctl.solver().removeConstraintSet(*workspaceConstraint_);
  }
  if(upVectorTask_)
  {
    ctl.solver().removeTask(upVectorTask_);
    upVectorTask_.reset();
  }
  if(eeTask_)
  {
    ctl.solver().removeTask(eeTask_);
    eeTask_.reset();
  }
  if(postureTask_)
  {
    ctl.solver().removeTask(postureTask_);
    postureTask_.reset();
  }
  workspaceConstraint_.reset();

  auto defaultPostureTask = ctl.getPostureTask(ctl.robot().name());
  defaultPostureTask->reset();
  ctl.solver().addTask(defaultPostureTask);

  const bool estimatorIsActive = ctl.datastore().call<bool>("EF_Estimator::isActive");
  if(estimatorIsActive != estimatorWasActive_)
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  const bool estimatorUsesForceSensor = ctl.datastore().call<bool>("EF_Estimator::useForceSensor");
  if(estimatorUsesForceSensor != estimatorWasUsingForceSensor_)
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
}

void BoxDemoController_Compliance::setupWorkspaceGUI(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->addElement(
      this, {"Controller", "Workspace"},
      mc_rtc::gui::Checkbox(
          "Enabled", [this]() { return workspaceConstraintEnabled_; },
          [this, &ctl]() { setWorkspaceConstraintEnabled(ctl, !workspaceConstraintEnabled_); }),
      mc_rtc::gui::ArrayInput(
          "Lower bound", {"x-", "y-", "z-"}, [this]() { return workspaceMin_; },
          [this, &ctl](const Eigen::Vector3d & value) { setWorkspaceBounds(ctl, value, workspaceMax_); }),
      mc_rtc::gui::ArrayInput(
          "Upper bound", {"x+", "y+", "z+"}, [this]() { return workspaceMax_; },
          [this, &ctl](const Eigen::Vector3d & value) { setWorkspaceBounds(ctl, workspaceMin_, value); }),
      mc_rtc::gui::NumberInput(
          "Safety distance ds", [this]() { return workspaceSafetyDistance(); },
          [this, &ctl](double value) { setWorkspaceSafetyDistance(ctl, value); }),
      mc_rtc::gui::NumberInput(
          "Closed-loop damper m", [this]() { return workspaceDamperM(); },
          [this, &ctl](double value) { setWorkspaceDamperM(ctl, value); }),
      mc_rtc::gui::NumberInput(
          "Closed-loop damper lambda", [this]() { return workspaceDamperLambda(); },
          [this, &ctl](double value) { setWorkspaceDamperLambda(ctl, value); }),
      mc_rtc::gui::Label("Status", [this, &ctl]() { return workspaceStatus(ctl); }),
      mc_rtc::gui::ArrayLabel("Slack", {"x", "y", "z"}, [this, &ctl]() { return workspaceSlack(ctl); }),
      mc_rtc::gui::Point3D("Tool position", [this, &ctl]() { return toolPosition(ctl); }),
      mc_rtc::gui::Point3D("Workspace center", [this]() { return workspaceCenter(); }),
      mc_rtc::gui::Visual(
          "Workspace box",
          [this, &ctl]()
          {
            const auto color = !workspaceConstraintEnabled_  ? mc_rtc::gui::Color{0.4, 0.4, 0.4, 0.15}
                               : !toolInWorkspace(ctl)       ? mc_rtc::gui::Color{0.9, 0.2, 0.2, 0.3}
                               : toolPastSafetyDistance(ctl) ? mc_rtc::gui::Color{0.95, 0.55, 0.1, 0.28}
                                                             : mc_rtc::gui::Color{0.1, 0.7, 0.2, 0.25};
            return mc_rtc::makeVisualBox(workspaceSize(), color);
          },
          [this]() { return workspacePose(); }),
      mc_rtc::gui::Visual(
          "Safety region", [this]()
          { return mc_rtc::makeVisualBox(workspaceInnerSize(workspaceSafetyDistance()), {1.0, 0.1, 0.1, 0.08}); },
          [this]() { return workspacePose(); }));
}

void BoxDemoController_Compliance::addComplianceLogs(mc_control::fsm::Controller & ctl)
{
  ctl.logger().addLogEntries(
      this, "BoxDemoCtrl_workspaceEnabled", [this]() { return workspaceConstraintEnabled_; },
      "BoxDemoCtrl_workspaceMin", [this]() { return workspaceMin_; }, "BoxDemoCtrl_workspaceMax", [this]()
      { return workspaceMax_; }, "BoxDemoCtrl_workspaceSafetyDistance", [this]() { return workspaceSafetyDistance(); },
      "BoxDemoCtrl_workspaceDamperM", [this]() { return workspaceDamperM(); }, "BoxDemoCtrl_workspaceDamperLambda",
      [this]() { return workspaceDamperLambda(); }, "BoxDemoCtrl_workspaceCenter",
      [this]() { return workspaceCenter(); }, "BoxDemoCtrl_workspaceSize", [this]() { return workspaceSize(); },
      "BoxDemoCtrl_toolPosition", [this, &ctl]() { return toolPosition(ctl); }, "BoxDemoCtrl_workspaceSlack",
      [this, &ctl]() { return workspaceSlack(ctl); }, "BoxDemoCtrl_toolInWorkspace",
      [this, &ctl]() { return toolInWorkspace(ctl); }, "BoxDemoCtrl_toolPastSafetyDistance",
      [this, &ctl]() { return toolPastSafetyDistance(ctl); }, "BoxDemoCtrl_stop", [this]() { return stop; },
      "BoxDemoCtrl_residualGain", [this]() { return residualGain; }, "BoxDemoCtrl_FsSensorRaw",
      [this]() { return FsSensorRaw; }, "BoxDemoCtrl_FsRaw", [this]() { return FsRaw; }, "BoxDemoCtrl_Fs", [this]()
      { return Fs; }, "BoxDemoCtrl_tau_s", [this]() { return tau_s; }, "BoxDemoCtrl_AsRaw", [this]() { return AsRaw; },
      "BoxDemoCtrl_As", [this]() { return As; }, "BoxDemoCtrl_tau_r", [this]() { return tau_r; },
      "BoxDemoCtrl_tau_r_aug", [this]() { return tau_r_aug; }, "BoxDemoCtrl_Fr", [this]() { return Fr; },
      "BoxDemoCtrl_Fr_aug", [this]() { return Fr_aug; }, "BoxDemoCtrl_Ar", [this]() { return Ar; },
      "BoxDemoCtrl_Ar_aug", [this]() { return Ar_aug; }, "BoxDemoCtrl_dF", [this]() { return dF; }, "BoxDemoCtrl_dA",
      [this]() { return dA; }, "BoxDemoCtrl_comp", [this]() { return comp; }, "BoxDemoCtrl_refAccelRaw", [this]()
      { return refAccelRaw; }, "BoxDemoCtrl_exclusiveComplianceModes", [this]() { return exclusiveComplianceModes; },
      "BoxDemoCtrl_triggerAveragingWindow", [this]() { return triggerAveragingWindow; }, "BoxDemoCtrl_refAccel",
      [this]() { return refAccel; }, "BoxDemoCtrl_orientationManagementMode",
      [this]() { return orientationManagementModeName(orientationManagementMode); }, "BoxDemoCtrl_localYAngle",
      [this]() { return localYOrientationAngle; });
}

void BoxDemoController_Compliance::setWorkspaceConstraintEnabled(mc_control::fsm::Controller & ctl, bool enabled)
{
  if(workspaceConstraintEnabled_ == enabled)
  {
    return;
  }
  workspaceConstraintEnabled_ = enabled;
  if(!workspaceConstraint_)
  {
    return;
  }
  if(workspaceConstraintEnabled_)
  {
    if(!workspaceConstraint_->inSolver())
    {
      ctl.solver().addConstraintSet(*workspaceConstraint_);
    }
  }
  else if(workspaceConstraint_->inSolver())
  {
    ctl.solver().removeConstraintSet(*workspaceConstraint_);
  }
}

void BoxDemoController_Compliance::setWorkspaceBounds(mc_control::fsm::Controller & ctl,
                                                      const Eigen::Vector3d & lower,
                                                      const Eigen::Vector3d & upper)
{
  workspaceMin_ = lower.cwiseMin(upper);
  workspaceMax_ = lower.cwiseMax(upper);
  if(workspaceConstraint_)
  {
    workspaceConstraint_->bounds(ctl.solver(), workspaceMin_, workspaceMax_);
  }
}

double BoxDemoController_Compliance::workspaceDamperM() const
{
  return workspaceConstraint_ ? workspaceConstraint_->m() : 1.0;
}

double BoxDemoController_Compliance::workspaceDamperLambda() const
{
  return workspaceConstraint_ ? workspaceConstraint_->lambda() : 10.0;
}

void BoxDemoController_Compliance::setWorkspaceDamperM(mc_control::fsm::Controller & ctl, double value)
{
  if(workspaceConstraint_)
  {
    workspaceConstraint_->m(ctl.solver(), value);
  }
}

void BoxDemoController_Compliance::setWorkspaceDamperLambda(mc_control::fsm::Controller & ctl, double value)
{
  if(workspaceConstraint_)
  {
    workspaceConstraint_->lambda(ctl.solver(), value);
  }
}

double BoxDemoController_Compliance::workspaceSafetyDistance() const
{
  return workspaceConstraint_ ? workspaceConstraint_->safetyDistance() : 0.005;
}

void BoxDemoController_Compliance::setWorkspaceSafetyDistance(mc_control::fsm::Controller & ctl, double value)
{
  if(workspaceConstraint_)
  {
    workspaceConstraint_->safetyDistance(ctl.solver(), value);
  }
}

Eigen::Vector3d BoxDemoController_Compliance::workspaceCenter() const
{
  return 0.5 * (workspaceMin_ + workspaceMax_);
}

Eigen::Vector3d BoxDemoController_Compliance::workspaceSize() const
{
  return workspaceMax_ - workspaceMin_;
}

Eigen::Vector3d BoxDemoController_Compliance::workspaceInnerSize(double margin) const
{
  return (workspaceSize() - Eigen::Vector3d::Constant(2.0 * std::max(0.0, margin))).cwiseMax(0.0);
}

sva::PTransformd BoxDemoController_Compliance::workspacePose() const
{
  return sva::PTransformd(Eigen::Matrix3d::Identity(), workspaceCenter());
}

Eigen::Vector3d BoxDemoController_Compliance::toolPosition(const mc_control::fsm::Controller & ctl) const
{
  return ctl.robot().frame(toolFrameName_).position().translation();
}

Eigen::Vector3d BoxDemoController_Compliance::workspaceSlack(const mc_control::fsm::Controller & ctl) const
{
  const Eigen::Vector3d position = toolPosition(ctl);
  return (position - workspaceMin_).cwiseMin(workspaceMax_ - position);
}

bool BoxDemoController_Compliance::toolInWorkspace(const mc_control::fsm::Controller & ctl) const
{
  const Eigen::Vector3d position = toolPosition(ctl);
  return (position.array() >= workspaceMin_.array()).all() && (position.array() <= workspaceMax_.array()).all();
}

bool BoxDemoController_Compliance::toolPastSafetyDistance(const mc_control::fsm::Controller & ctl) const
{
  return toolInWorkspace(ctl) && workspaceSlack(ctl).minCoeff() < workspaceSafetyDistance();
}

std::string BoxDemoController_Compliance::workspaceStatus(const mc_control::fsm::Controller & ctl) const
{
  if(!workspaceConstraintEnabled_)
  {
    return "Disabled";
  }
  if(!toolInWorkspace(ctl))
  {
    return "Outside";
  }
  return toolPastSafetyDistance(ctl) ? "Warning" : "Inside";
}

Eigen::Matrix3d BoxDemoController_Compliance::targetOrientation() const
{
  const Eigen::Vector4d quaternion = targetOrientationQuaternion_.squaredNorm() < 1e-12
                                         ? Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)
                                         : targetOrientationQuaternion_;
  return Eigen::Quaterniond(quaternion[0], quaternion[1], quaternion[2], quaternion[3]).normalized().toRotationMatrix();
}

EXPORT_SINGLE_STATE("BoxDemoController_Compliance", BoxDemoController_Compliance)
