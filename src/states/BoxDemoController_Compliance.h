#pragma once

#include <deque>
#include <map>
#include <mc_control/fsm/State.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <mc_tasks/VectorOrientationTask.h>
#include <memory>
#include <string>
#include <vector>

struct BoxDemoController_WorkspaceConstraint;

struct BoxDemoController_Compliance_AdjustableVectorOrientationTask : public mc_tasks::VectorOrientationTask
{
  using mc_tasks::VectorOrientationTask::VectorOrientationTask;

  void frameVector(const Eigen::Vector3d & vector);
};

struct BoxDemoController_Compliance : mc_control::fsm::State
{
  enum class OrientationManagementMode
  {
    Compliance,
    WorldZTorqueToLocalY
  };

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
  double planarTranslationStiffness = 0.0;
  Eigen::Vector3d planarTranslationDamping = Eigen::Vector3d::Constant(10.0);
  double currentPositionStiffness = nominalPositionStiffness;
  Eigen::Vector3d currentPositionDamping = Eigen::Vector3d::Constant(nominalPositionDamping);
  Eigen::Vector3d nominalOrientationStiffness = Eigen::Vector3d::Constant(100.0);
  Eigen::Vector3d nominalOrientationDamping = Eigen::Vector3d::Constant(40.0);
  double rotationZOrientationStiffness = 10.0;
  double rotationZOrientationDamping = 40.0;
  double worldZTorqueToLocalYGain = 0.02;
  double maxLocalYOrientationOffset = 0.35;
  Eigen::Vector3d currentOrientationStiffness = nominalOrientationStiffness;
  Eigen::Vector3d currentOrientationDamping = nominalOrientationDamping;
  double gainTransitionTime = 0.2;
  double planarTranslationTriggerOnThreshold = 10.0;
  double planarTranslationTriggerOffThreshold = 8;
  double rotationZTriggerOnThreshold = 10.0;
  double rotationZTriggerOffThreshold = 8.0;
  double dFZComplianceOnThreshold = 5.0;
  double dFZComplianceOffThreshold = 3.0;
  double triggerAveragingWindow = 0.0;
  double complianceGain = 0.8;
  double residualGain = 10.0;
  Eigen::Matrix3d baseOrientationTarget = Eigen::Matrix3d::Identity();
  double localYOrientationAngle = 0.0;
  OrientationManagementMode orientationManagementMode = OrientationManagementMode::Compliance;
  bool refAccelFilterInitialized = false;
  bool planarTranslationTriggerActive = false;
  bool rotationZTriggerActive = false;
  bool dFZComplianceActive = false;
  bool exclusiveComplianceModes = false;
  bool updateGain = false;
  std::deque<double> planarTranslationTriggerHistory;
  std::deque<double> rotationZTriggerHistory;
  std::deque<double> dFZComplianceHistory;
  double planarTranslationTriggerSum = 0.0;
  double rotationZTriggerSum = 0.0;
  double dFZComplianceSum = 0.0;

  static std::string orientationManagementModeName(OrientationManagementMode mode);
  static OrientationManagementMode orientationManagementModeFromName(const std::string & mode);
  static void updateSlidingAverage(std::deque<double> & history, double & sum, double value, size_t maxSamples);

  void setupWorkspaceGUI(mc_control::fsm::Controller & ctl);
  void addComplianceLogs(mc_control::fsm::Controller & ctl);
  void setWorkspaceConstraintEnabled(mc_control::fsm::Controller & ctl, bool enabled);
  void setWorkspaceBounds(mc_control::fsm::Controller & ctl,
                          const Eigen::Vector3d & lower,
                          const Eigen::Vector3d & upper);
  double workspaceDamperM() const;
  double workspaceDamperLambda() const;
  void setWorkspaceDamperM(mc_control::fsm::Controller & ctl, double value);
  void setWorkspaceDamperLambda(mc_control::fsm::Controller & ctl, double value);
  double workspaceSafetyDistance() const;
  void setWorkspaceSafetyDistance(mc_control::fsm::Controller & ctl, double value);
  Eigen::Vector3d workspaceCenter() const;
  Eigen::Vector3d workspaceSize() const;
  Eigen::Vector3d workspaceInnerSize(double margin) const;
  sva::PTransformd workspacePose() const;
  Eigen::Vector3d toolPosition(const mc_control::fsm::Controller & ctl) const;
  Eigen::Vector3d workspaceSlack(const mc_control::fsm::Controller & ctl) const;
  bool toolInWorkspace(const mc_control::fsm::Controller & ctl) const;
  bool toolPastSafetyDistance(const mc_control::fsm::Controller & ctl) const;
  std::string workspaceStatus(const mc_control::fsm::Controller & ctl) const;
  Eigen::Matrix3d targetOrientation() const;

  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> eeTask_;
  std::shared_ptr<mc_tasks::CompliantPostureTask> postureTask_;
  std::shared_ptr<BoxDemoController_Compliance_AdjustableVectorOrientationTask> upVectorTask_;
  std::shared_ptr<BoxDemoController_WorkspaceConstraint> workspaceConstraint_;
  std::map<std::string, std::vector<double>> postureHome_ = {{"joint_1", {0}},
                                                             {"joint_2", {0.262}},
                                                             {"joint_3", {3.14}},
                                                             {"joint_4", {-2.269}},
                                                             {"joint_5", {0}},
                                                             {"joint_6", {0.96}},
                                                             {"joint_7", {1.57}}};
  Eigen::Vector3d targetPosition_ = Eigen::Vector3d(0.6, 0.0, 0.4);
  Eigen::Vector4d targetOrientationQuaternion_ = Eigen::Vector4d(1.0, -1.0, -1.0, -1.0);
  std::string toolFrameName_ = "tool";
  bool workspaceConstraintEnabled_ = true;
  Eigen::Vector3d workspaceMin_ = Eigen::Vector3d(0.4, -0.3, 0.2);
  Eigen::Vector3d workspaceMax_ = Eigen::Vector3d(0.8, 0.3, 0.6);
  Eigen::Vector3d nominalUpVectorInTool_ = Eigen::Vector3d::UnitZ();
  bool estimatorWasActive_ = false;
  bool estimatorWasUsingForceSensor_ = false;
};
