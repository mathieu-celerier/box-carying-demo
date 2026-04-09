#include "BoxDemoController.h"

BoxDemoController::BoxDemoController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  // auto robot_config = config(robot().module().name);
  std::string tool_frame = "tool";
  // tool_frame = robot_config("tool_frame", (std::string) "");
  // mc_rtc::log::info("Using tool frame: {}", tool_frame);

  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 20});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(
      robots(), robot().robotIndex(), solver().dt(), std::array<double, 3>({0.1, 0.01, 0.5}),
      std::array<double, 2>({1.2, 70}), 0.9, false, true));
  solver().addConstraintSet(dynamicsConstraint);

  postureHome = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                 {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};

  eeTask = std::make_shared<mc_tasks::CompliantEndEffectorTask>(tool_frame, robots(), robot().robotIndex(), 2, 10000);
  eeTask->orientationTask->orientation(Eigen::Quaterniond(1.2, -0.8, -0.8, -1.2).normalized().toRotationMatrix());
  eeTask->positionTask->position(Eigen::Vector3d(0.6, 0.0, 0.4));
  solver().addTask(eeTask);

  solver().removeTask(getPostureTask(robot().name()));
  postureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1, 1);
  postureTask->reset();
  postureTask->stiffness(1.0);
  postureTask->damping(4.0);
  postureTask->target(postureHome);
  postureTask->makeCompliant(true);
  solver().addTask(postureTask);

  datastore().make<std::string>("ControlMode", "Position");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });

  mc_rtc::log::success("BoxDemoController init done ");
}

bool BoxDemoController::run()
{
  // Update the solver depending on the control mode
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
}

void BoxDemoController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}
