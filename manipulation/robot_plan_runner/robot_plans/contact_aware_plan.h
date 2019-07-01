#pragma once

#include <Eigen/Core>

#include "drake/manipulation/robot_plan_runner/robot_plans/task_space_plan.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

class ContactAwarePlan : public TaskSpacePlan {
 public:
  ContactAwarePlan();

  void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
            const Eigen::Ref<const Eigen::VectorXd>& v,
            const Eigen::Ref<const Eigen::VectorXd>& tau_external,
            double control_period, double t, const PlanData& plan_data,
            EigenPtr<Eigen::VectorXd> q_cmd,
            EigenPtr<Eigen::VectorXd> tau_cmd) const override;

 private:
  void UpdatePositionError(
      double t, const PlanData& plan_data,
      const Eigen::Ref<const Eigen::Vector3d>& p_WoQ_W) const override;
  /*
   * pC_B: coordinate of the contact point in the frame of the body under
   *   contact. For now it is assumed to be the same as the task frame.
   * tau_external: external joint torque estimated by the robot.
   */
  double EstimateContactForceNorm(
      const Eigen::Ref<const Eigen::Vector3d>& pC_B,
      const Eigen::Ref<const Eigen::VectorXd>& tau_external) const;


  Eigen::ArrayXd joint_stiffness_;
  double velocity_cost_weight_;

  std::unique_ptr<solvers::MathematicalProgramResult> prog_result_;
  solvers::GurobiSolver solver_;
//
//  std::unique_ptr<solvers::MathematicalProgram> prog_;
//  solvers::VectorXDecisionVariable dq_;
//  solvers::LinearEqualityConstraint* ee_task_constraint_{nullptr};
};



}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
