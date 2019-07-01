
#include "drake/manipulation/robot_plan_runner/robot_plans/contact_aware_plan.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using std::cout;
using std::endl;

ContactAwarePlan::ContactAwarePlan() : TaskSpacePlan() {
  this->set_plan_type(PlanType::kContactAwarePlan);
  if (!solver_.available()) {
    throw std::runtime_error("Gurobi solver is not available.");
  }

  joint_stiffness_.resize(num_positions_);
  joint_stiffness_ << 800, 600, 600, 600, 400, 200, 200;
  velocity_cost_weight_ = 0.1;

  // MahtematicalProgram-related declarations.
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
}

void ContactAwarePlan::UpdatePositionError(
    double, const PlanData& plan_data,
    const Eigen::Ref<const Eigen::Vector3d>& p_WoQ_W) const {
  if (!p_WoQ_W_t0_) {
    p_WoQ_W_t0_ = std::make_unique<Eigen::Vector3d>(p_WoQ_W);
  }

  const double t_final = plan_data.ee_data.value().ee_xyz_traj.end_time();
  const auto p_WoQ_W_ref =
      plan_data.ee_data.value().ee_xyz_traj.value(t_final) + *p_WoQ_W_t0_;
  err_xyz_ = p_WoQ_W_ref - p_WoQ_W;

  const double err_norm = p_WoQ_W_ref.norm();
  const double err_norm_max = 0.05;
  if (err_norm > err_norm_max) {
    err_xyz_ *= err_norm_max / err_norm;
  }
}

double ContactAwarePlan::EstimateContactForceNorm(
    const Eigen::Ref<const Eigen::Vector3d>& pC_T,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external) const {
  Eigen::MatrixXd Jv_WTc(3, num_positions_);  // contact jacobian
  Eigen::Vector3d p_WC;
  plant_->CalcPointsGeometricJacobianExpressedInWorld(
      *plant_context_, plant_->get_frame(task_frame_idx_), pC_T, &p_WC,
      &Jv_WTc);

  // least square solve Jv_WTc.T.dot(f) = tau_external.
  auto svd =
      Jv_WTc.transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::Vector3d f = svd.solve(tau_external);

  return f.norm();
}

void ContactAwarePlan::Step(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
    double control_period, double t, const PlanData& plan_data,
    EigenPtr<Eigen::VectorXd> q_cmd, EigenPtr<Eigen::VectorXd> tau_cmd) const {
  this->check_plan_type(plan_data);

  // Update q and v in plant_context_, which is owned by this class.
  plant_->SetPositions(plant_context_.get(), robot_model_, q);
  plant_->SetVelocities(plant_context_.get(), robot_model_, v);

  // Update Kinematics.
  const auto X_WT =
      plant_->CalcRelativeTransform(*plant_context_, plant_->world_frame(),
                                    plant_->get_frame(task_frame_idx_));
  const auto& p_ToQ_T = plan_data.ee_data.value().p_ToQ_T;
  const auto p_WoQ_W = X_WT * p_ToQ_T;
  const auto Q_WT = X_WT.rotation().ToQuaternion();

  plant_->CalcFrameGeometricJacobianExpressedInWorld(
      *plant_context_, plant_->get_frame(task_frame_idx_), p_ToQ_T, &Jv_WTq_);

  // Update errors.
  this->UpdatePositionError(t, plan_data, p_WoQ_W);
  this->UpdateOrientationError(t, plan_data, Q_WT);

  // Update x_dot_desired.
  x_dot_desired_.tail(3) = kp_translation * err_xyz_.array();
  x_dot_desired_.head(3) = Q_WT * (kp_rotation * Q_TTr_.vec().array()).matrix();

  // Deal with contact.
  if (tau_external.cwiseAbs().maxCoeff() > 3) {
    // assuming the contact is at the center of the sphere.
    Eigen::Vector3d pC_T(0, 0, 0.075);
    const double f_norm =
        this->EstimateContactForceNorm(pC_T, tau_external);
    std::cout << f_norm << endl;
    throw std::runtime_error("I've seen enough.");
  }

  // Update coefficients of QP.
  ee_task_constraint_->UpdateCoefficients(Jv_WTq_, x_dot_desired_);
  solver_.Solve(*prog_, {}, {}, prog_result_.get());

  if (!prog_result_->is_success()) {
    throw std::runtime_error("Controller QP cannot be solved.");
  }
  auto q_dot_desired_value = prog_result_->GetSolution(q_dot_desired_);
  *q_cmd = q + q_dot_desired_value * control_period;
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
