#pragma once

#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_utilities.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

class JointSpacePlanContact : public PlanBase {
 public:
  explicit JointSpacePlanContact(int num_positions);

  void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
            const Eigen::Ref<const Eigen::VectorXd>& v,
            const Eigen::Ref<const Eigen::VectorXd>& tau_external,
            double control_period, double t, const PlanData& plan_data,
            EigenPtr<Eigen::VectorXd> q_cmd,
            EigenPtr<Eigen::VectorXd> tau_cmd) const override;

 private:
  Eigen::ArrayXd joint_stiffness_;
  const double velocity_cost_weight_;
  Eigen::MatrixXd dq_weight_;

  std::unique_ptr<ContactForceEstimator> contact_force_estimator_;
  std::unique_ptr<solvers::MathematicalProgramResult> prog_result_;
  solvers::GurobiSolver solver_;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
