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
  void UpdatePositionErrorUsingTargetPoint(
      double t, const PlanData& plan_data,
      const Eigen::Ref<const Eigen::Vector3d>& p_WoQ_W) const;
  /*
   * pC_B: coordinate of the contact point in the frame of the body under
   *   contact. For now it is assumed to be the same as the task frame.
   * tau_external: external joint torque estimated by the robot.
   */
  Eigen::Vector3d EstimateContactForce(
      const Eigen::Ref<const Eigen::Vector3d>& pC_T,
      const Eigen::Ref<const Eigen::VectorXd>& tau_external) const;

  void LowPassFilterContactForce(
      const Eigen::Ref<const Eigen::Vector3d> f_new, double h) const;

  // filtered contact force
  mutable std::unique_ptr<Eigen::Vector3d> f_filtered_;

  // cutoff frequency of the low-pass filter on contact force, in rad/s.
  const double w_cutoff_;

  // Contact Jacobian (3 * num_positions_);
  mutable Eigen::MatrixXd Jv_WTc_;

  Eigen::ArrayXd joint_stiffness_;
  const double velocity_cost_weight_;
  Eigen::MatrixXd dq_weight_;

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
