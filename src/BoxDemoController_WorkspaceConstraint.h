#pragma once

#include <mc_rbdyn/fwd.h>
#include <mc_solver/ConstraintSet.h>

#include <mc_rtc/void_ptr.h>

struct BoxDemoController_WorkspaceConstraint : public mc_solver::ConstraintSet
{
  BoxDemoController_WorkspaceConstraint(const mc_rbdyn::RobotFrame & frame,
                                        const Eigen::Vector3d & lower,
                                        const Eigen::Vector3d & upper);

  const Eigen::Vector3d & lower() const;
  const Eigen::Vector3d & upper() const;
  void bounds(mc_solver::QPSolver & solver, const Eigen::Vector3d & lower, const Eigen::Vector3d & upper);
  double m() const;
  double lambda() const;
  double safetyDistance() const;
  void safetyDistance(mc_solver::QPSolver & solver, double value);
  void m(mc_solver::QPSolver & solver, double value);
  void lambda(mc_solver::QPSolver & solver, double value);

protected:
  void addToSolverImpl(mc_solver::QPSolver & solver) override;
  void removeFromSolverImpl(mc_solver::QPSolver & solver) override;

private:
  mc_rtc::void_ptr constraint_;
};
