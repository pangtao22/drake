
#include "drake/manipulation/robot_plan_runner/robot_plans/contact_aware_plan.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

ContactAwarePlan::ContactAwarePlan() : TaskSpacePlan() {
  this->set_plan_type(PlanType::kContactAwarePlan);
  if (!solver_.available()) {
    throw std::runtime_error("Gurobi solver is not available.");
  }

  prog_ = std::make_unique<solvers::MathematicalProgram>();
  prog_result_ = std::make_unique<solvers::MathematicalProgramResult>();

  q_dot_desired_ = prog_->NewContinuousVariables(num_positions_);
  ee_task_constraint_ =
      prog_
          ->AddLinearEqualityConstraint(
              Eigen::MatrixXd::Zero(task_dimension_, num_positions_),
              Eigen::VectorXd::Zero(task_dimension_), q_dot_desired_)
          .evaluator()
          .get();
  prog_->AddL2NormCost(
      Eigen::MatrixXd::Identity(num_positions_, num_positions_),
      Eigen::VectorXd::Zero(num_positions_), q_dot_desired_);
};

void ContactAwarePlan::Step(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>&,
    double control_period, double t, const PlanData& plan_data,
    EigenPtr<Eigen::VectorXd> q_cmd,
    EigenPtr<Eigen::VectorXd> tau_cmd) const {
  if(plan_data.plan_type != this->get_plan_type()) {
    throw std::runtime_error("Mismatch between Plan and PlanData.");
  }
  this->UpdateDesiredTaskSpaceVelocity(q, v, t, plan_data);

  ee_task_constraint_->UpdateCoefficients(Jv_WTq_, x_dot_desired_);
  solver_.Solve(*prog_, {}, {}, prog_result_.get());

  if(!prog_result_->is_success()) {
    throw std::runtime_error("Controller QP cannot be solved.");
  }
  auto q_dot_desired_value = prog_result_->GetSolution(q_dot_desired_);
  *q_cmd = q + q_dot_desired_value * control_period;
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
