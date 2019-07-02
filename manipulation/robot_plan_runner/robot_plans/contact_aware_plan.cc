
#include "drake/manipulation/robot_plan_runner/robot_plans/contact_aware_plan.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using std::cout;
using std::endl;

ContactAwarePlan::ContactAwarePlan()
    : TaskSpacePlan(), w_cutoff_(0.5 * 2 * M_PI), velocity_cost_weight_(0.1) {
  this->set_plan_type(PlanType::kContactAwarePlan);
  if (!solver_.available()) {
    throw std::runtime_error("Gurobi solver is not available.");
  }

  joint_stiffness_.resize(num_positions_);
  joint_stiffness_ << 800, 600, 600, 600, 400, 200, 200;

  dq_weight_.resize(num_positions_, num_positions_);
  dq_weight_.setZero();
  dq_weight_.diagonal() << 10, 9, 8, 8, 7, 2, 1;

  // contact Jacobian.
  Jv_WTc_.resize(3, num_positions_);

  prog_result_ = std::make_unique<solvers::MathematicalProgramResult>();
}

void ContactAwarePlan::UpdatePositionErrorUsingTargetPoint(
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
  const double err_norm_max = 0.01;
  if (err_norm > err_norm_max) {
    err_xyz_ *= err_norm_max / err_norm;
  }
}

Eigen::Vector3d ContactAwarePlan::EstimateContactForce(
    const Eigen::Ref<const Eigen::Vector3d>& pC_T,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external) const {
  // updates contact jacobian.
  Eigen::Vector3d p_WC;
  plant_->CalcPointsGeometricJacobianExpressedInWorld(
      *plant_context_, plant_->get_frame(task_frame_idx_), pC_T, &p_WC,
      &Jv_WTc_);

  // least square solve Jv_WTc.T.dot(f) = tau_external.
  auto svd =
      Jv_WTc_.transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  return svd.solve(tau_external);
}

void ContactAwarePlan::LowPassFilterContactForce(
    const Eigen::Ref<const Eigen::Vector3d> f_new, double h) const {
  const double a = h * w_cutoff_ / (1 + h * w_cutoff_);
  if (f_filtered_) {
    *f_filtered_ = (1 - a) * (*f_filtered_) + a * f_new;
  } else {
    f_filtered_.reset(new Eigen::Vector3d(f_new));
  }
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

  // Update contact force estimation (f_filtered_), assuming the contact is at
  // the center of the sphere.
  const Eigen::Vector3d pC_T(0, 0, 0.075);
  const Eigen::Vector3d f_contact =
      this->EstimateContactForce(pC_T, tau_external);
  this->LowPassFilterContactForce(f_contact, control_period);

  // MahtematicalProgram-related declarations.
  const auto prog = std::make_unique<solvers::MathematicalProgram>();
  auto dq = prog->NewContinuousVariables(num_positions_);

  // alias for task Jacobian.
  const Eigen::MatrixXd& Jt = Jv_WTq_;

  // tracking error cost
  prog->AddL2NormCost(Jt / control_period, x_dot_desired_, dq);

  // joint velocity cost
  prog->AddQuadraticErrorCost(
      velocity_cost_weight_ / std::pow(control_period, 2) * dq_weight_,
      Eigen::VectorXd::Zero(num_positions_), dq);

  // Deal with contact.
  const double f_norm_threshold = 10;
  const double f_norm = (*f_filtered_).norm();
  if (f_norm > f_norm_threshold) {
    Eigen::Vector3d contact_normal = *f_filtered_ / f_norm;

    const Eigen::RowVectorXd J_nc = contact_normal.transpose() * Jv_WTc_;
    const Eigen::MatrixXd J_nc_pinv =
        J_nc.completeOrthogonalDecomposition().pseudoInverse();

    // force constraint
    const Eigen::Matrix<double, 1, 1> f_desired(f_norm_threshold * 1.5);
    prog->AddLinearEqualityConstraint(
        (J_nc_pinv.array() * joint_stiffness_).matrix().transpose(), -f_desired,
        dq);

    // contact-maintaining constraint
    prog->AddLinearConstraint(J_nc / control_period,
                              -std::numeric_limits<double>::infinity(), 0, dq);
  }

  // Update coefficients of QP.
  //  ee_task_constraint_->UpdateCoefficients(Jv_WTq_, x_dot_desired_);
  solver_.Solve(*prog, {}, {}, prog_result_.get());

  //  *prog_result_ = solvers::Solve(*prog);

  if (!prog_result_->is_success()) {
    throw std::runtime_error("Controller QP cannot be solved.");
  }
  auto dq_value = prog_result_->GetSolution(dq);
  *q_cmd = q + dq_value;
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
