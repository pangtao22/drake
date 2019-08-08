#pragma once

#include <Eigen/Core>

#include "drake/manipulation/robot_plan_runner/robot_plans/task_space_plan.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

class HybridForcePositionPlan : public TaskSpacePlan {
 public:
  HybridForcePositionPlan();

  void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
            const Eigen::Ref<const Eigen::VectorXd>& v,
            const Eigen::Ref<const Eigen::VectorXd>& tau_external,
            double control_period, double t, const PlanData& plan_data,
            const robot_plans::ContactInfo& contact_info,
            EigenPtr<Eigen::VectorXd> q_cmd,
            EigenPtr<Eigen::VectorXd> tau_cmd) const override;

 private:
  const double velocity_cost_weight_;
  const double f_contact_growth_rate_;
  const double f_contact_desired_;
  mutable double f_contact_;
  Eigen::ArrayXd joint_stiffness_;
  Eigen::MatrixXd dq_weight_;

  std::unique_ptr<solvers::MathematicalProgramResult> prog_result_;
  solvers::GurobiSolver solver_;
};


}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake