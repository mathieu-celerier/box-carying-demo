#include "BoxDemoController_WorkspaceConstraint.h"

#include <mc_rbdyn/RobotFrame.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

#include <tvm/requirements/PriorityLevel.h>
#include <tvm/task_dynamics/VelocityDamper.h>

namespace
{

struct WorkspaceFramePositionFunction : public tvm::function::abstract::Function
{
  SET_UPDATES(WorkspaceFramePositionFunction, Value, Velocity, Jacobian, NormalAcceleration)

  WorkspaceFramePositionFunction(const mc_rbdyn::RobotFrame & frame)
  : tvm::function::abstract::Function(3), robotFrame_(&frame), frame_(frame.tvm_frame())
  {
    registerUpdates(Update::Value, &WorkspaceFramePositionFunction::updateValue, Update::Velocity,
                    &WorkspaceFramePositionFunction::updateVelocity, Update::Jacobian,
                    &WorkspaceFramePositionFunction::updateJacobian, Update::NormalAcceleration,
                    &WorkspaceFramePositionFunction::updateNormalAcceleration);
    addOutputDependency<WorkspaceFramePositionFunction>(Output::Value, Update::Value);
    addOutputDependency<WorkspaceFramePositionFunction>(Output::Velocity, Update::Velocity);
    addOutputDependency<WorkspaceFramePositionFunction>(Output::Jacobian, Update::Jacobian);
    addOutputDependency<WorkspaceFramePositionFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
    auto & robot = robotFrame_->robot().tvmRobot();
    addVariable(robot.q(), false);
    addInputDependency<WorkspaceFramePositionFunction>(Update::Value, frame_, mc_tvm::RobotFrame::Output::Position);
    addInputDependency<WorkspaceFramePositionFunction>(Update::Velocity, frame_, mc_tvm::RobotFrame::Output::Velocity);
    addInputDependency<WorkspaceFramePositionFunction>(Update::Jacobian, frame_, mc_tvm::RobotFrame::Output::Jacobian);
    addInputDependency<WorkspaceFramePositionFunction>(Update::NormalAcceleration, frame_,
                                                       mc_tvm::RobotFrame::Output::NormalAcceleration);
  }

  void updateValue()
  {
    value_ = frame_.position().translation();
  }

  void updateVelocity()
  {
    velocity_ = frame_.velocity().linear();
  }

  void updateJacobian()
  {
    splitJacobian(frame_.jacobian().bottomRows<3>(), robotFrame_->robot().tvmRobot().q());
  }

  void updateNormalAcceleration()
  {
    normalAcceleration_ = frame_.normalAcceleration().linear();
  }

  const mc_rbdyn::RobotFrame * robotFrame_;
  mc_tvm::RobotFrame & frame_;
};

struct TVMBoxDemoController_WorkspaceConstraint
{
  TVMBoxDemoController_WorkspaceConstraint(const mc_rbdyn::RobotFrame & frame,
                                           const Eigen::Vector3d & lower,
                                           const Eigen::Vector3d & upper)
  : function_(std::make_shared<WorkspaceFramePositionFunction>(frame)), lower_(lower), upper_(upper),
    // Explicitly use the closed-loop second-order velocity damper variant.
    damperConfig_(Eigen::Vector3d::Constant(0.2),
                  Eigen::Vector3d::Constant(0.05),
                  Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Constant(0.5),
                  Eigen::Vector3d::Constant(1.2),
                  Eigen::Vector3d::Constant(20.0))
  {
  }

  void addToSolver(mc_solver::TVMQPSolver & solver)
  {
    constraint_ = solver.problem().add(lower_ <= function_ <= upper_,
                                       tvm::task_dynamics::VelocityDamper(solver.dt(), damperConfig_),
                                       {tvm::requirements::PriorityLevel(0)});
  }

  void removeFromSolver(mc_solver::TVMQPSolver & solver)
  {
    solver.problem().remove(*constraint_);
    constraint_.reset();
  }

  void bounds(const Eigen::Vector3d & lower, const Eigen::Vector3d & upper)
  {
    lower_ = lower;
    upper_ = upper;
  }

  double m() const
  {
    return damperConfig_.m_.size() ? damperConfig_.m_[0] : 0.0;
  }

  double lambda() const
  {
    return damperConfig_.lambda_.size() ? damperConfig_.lambda_[0] : 0.0;
  }

  void m(double value)
  {
    damperConfig_.m_ = Eigen::Vector3d::Constant(std::max(1.0, value));
  }

  void lambda(double value)
  {
    damperConfig_.lambda_ = Eigen::Vector3d::Constant(std::max(0.0, value));
  }

  void safetyDistance(double value)
  {
    const double clamped = std::max(0.0, value);
    damperConfig_.ds_ = Eigen::Vector3d::Constant(clamped);
    damperConfig_.di_ = damperConfig_.di_.cwiseMax(Eigen::Vector3d::Constant(clamped + 1e-4));
  }

  std::shared_ptr<WorkspaceFramePositionFunction> function_;
  Eigen::Vector3d lower_;
  Eigen::Vector3d upper_;
  tvm::task_dynamics::VelocityDamper::AnisotropicConfig damperConfig_;
  tvm::TaskWithRequirementsPtr constraint_;
};

} // namespace

BoxDemoController_WorkspaceConstraint::BoxDemoController_WorkspaceConstraint(const mc_rbdyn::RobotFrame & frame,
                                                                             const Eigen::Vector3d & lower,
                                                                             const Eigen::Vector3d & upper)
: constraint_(mc_rtc::make_void_ptr<TVMBoxDemoController_WorkspaceConstraint>(frame, lower, upper))
{
}

const Eigen::Vector3d & BoxDemoController_WorkspaceConstraint::lower() const
{
  return static_cast<const TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get())->lower_;
}

const Eigen::Vector3d & BoxDemoController_WorkspaceConstraint::upper() const
{
  return static_cast<const TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get())->upper_;
}

void BoxDemoController_WorkspaceConstraint::bounds(mc_solver::QPSolver & solver,
                                                   const Eigen::Vector3d & lower,
                                                   const Eigen::Vector3d & upper)
{
  auto * constraint = static_cast<TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get());
  const bool wasInSolver = inSolver();
  if(wasInSolver)
  {
    removeFromSolver(solver);
  }
  constraint->bounds(lower, upper);
  if(wasInSolver)
  {
    addToSolver(solver);
  }
}

double BoxDemoController_WorkspaceConstraint::m() const
{
  return static_cast<const TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get())->m();
}

double BoxDemoController_WorkspaceConstraint::lambda() const
{
  return static_cast<const TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get())->lambda();
}

double BoxDemoController_WorkspaceConstraint::safetyDistance() const
{
  auto * constraint = static_cast<const TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get());
  return constraint->damperConfig_.ds_.size() ? constraint->damperConfig_.ds_[0] : 0.0;
}

void BoxDemoController_WorkspaceConstraint::safetyDistance(mc_solver::QPSolver & solver, double value)
{
  auto * constraint = static_cast<TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get());
  const bool wasInSolver = inSolver();
  if(wasInSolver)
  {
    removeFromSolver(solver);
  }
  constraint->safetyDistance(value);
  if(wasInSolver)
  {
    addToSolver(solver);
  }
}

void BoxDemoController_WorkspaceConstraint::m(mc_solver::QPSolver & solver, double value)
{
  auto * constraint = static_cast<TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get());
  const bool wasInSolver = inSolver();
  if(wasInSolver)
  {
    removeFromSolver(solver);
  }
  constraint->m(value);
  if(wasInSolver)
  {
    addToSolver(solver);
  }
}

void BoxDemoController_WorkspaceConstraint::lambda(mc_solver::QPSolver & solver, double value)
{
  auto * constraint = static_cast<TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get());
  const bool wasInSolver = inSolver();
  if(wasInSolver)
  {
    removeFromSolver(solver);
  }
  constraint->lambda(value);
  if(wasInSolver)
  {
    addToSolver(solver);
  }
}

void BoxDemoController_WorkspaceConstraint::addToSolverImpl(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    case mc_solver::QPSolver::Backend::TVM:
      static_cast<TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get())
          ->addToSolver(mc_solver::tvm_solver(solver));
      break;
    default:
      mc_rtc::log::error_and_throw("[BoxDemoController_WorkspaceConstraint] Only TVM backend is implemented");
  }
}

void BoxDemoController_WorkspaceConstraint::removeFromSolverImpl(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    case mc_solver::QPSolver::Backend::TVM:
      static_cast<TVMBoxDemoController_WorkspaceConstraint *>(constraint_.get())
          ->removeFromSolver(mc_solver::tvm_solver(solver));
      break;
    default:
      mc_rtc::log::error_and_throw("[BoxDemoController_WorkspaceConstraint] Only TVM backend is implemented");
  }
}
