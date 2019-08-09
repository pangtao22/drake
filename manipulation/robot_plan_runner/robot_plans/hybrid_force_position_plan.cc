
#include "drake/manipulation/robot_plan_runner/robot_plans/hybrid_force_position_plan.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_utilities.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

HybridForcePositionPlan::HybridForcePositionPlan()
    : TaskSpacePlan(),
      velocity_cost_weight_(0.01),
      f_contact_growth_rate_(0.005 / (1 + 0.005)),
      f_contact_desired_(10),
      f_contact_ref_(0),
      f_integrator_state_(0) {
  DRAKE_THROW_UNLESS(solver_.available());
  this->set_plan_type(PlanType::kHybridForcePositionPlan);

  joint_stiffness_.resize(num_positions_);
  joint_stiffness_ << 800, 600, 600, 600, 400, 200, 200;

  dq_weight_.resize(num_positions_, num_positions_);
  dq_weight_.setZero();
  dq_weight_.diagonal() << 5, 4, 3, 3, 2, 2, 1;

  prog_result_ = std::make_unique<solvers::MathematicalProgramResult>();

  contact_force_estimator_ =
      std::make_unique<ContactForceEstimator>(0.005, 0.5 * 2 * M_PI);
}

void HybridForcePositionPlan::Step(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
    double control_period, double t, const PlanData& plan_data,
    const robot_plans::ContactInfo&, EigenPtr<Eigen::VectorXd> q_cmd,
    EigenPtr<Eigen::VectorXd> tau_cmd) const {
  DRAKE_THROW_UNLESS(plan_data.plan_type == plan_type_);

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

  // Alias for the rotation and translation part of the task frame Jacobian.
  const auto& J_rotation = Jv_WTq_.topRows(3);
  const auto& J_translation = Jv_WTq_.bottomRows(3);

  // Calculate force and motion Jacobians.
  MatrixXd Jm, Jf;
  const auto& motion_axes =
      plan_data.hybrid_task_definition.value().motion_controlled_axes;
  const auto& force_axes =
      plan_data.hybrid_task_definition.value().force_controlled_axes;
  const int n_motion = motion_axes.size();
  const int n_force = force_axes.size();

  const auto& R_WC = plan_data.hybrid_task_definition.value().R_WC;
  Jm.resize(n_motion, num_positions_);
  Jf.resize(n_force, num_positions_);

  for (int i = 0; i < n_motion; i++) {
    Jm.row(i) = R_WC.col(motion_axes[i]).transpose() * J_translation;
  }

  for (int i = 0; i < n_force; i++) {
    Jf.row(i) = R_WC.col(force_axes[i]).transpose() * J_translation;
  }

  // Calculate position error along position-controlled.
  const auto e_xyz_C = R_WC.transpose() * err_xyz_;
  // x_dot_desired is expressed in frame C along the motion-controlled axes.
  VectorXd x_dot_desired;
  x_dot_desired.resize(n_motion);
  for (int i = 0; i < n_motion; i++) {
    x_dot_desired(i) = kp_translation_[i] * e_xyz_C(motion_axes[i]);
  }

  // Calculate orientation error.
  const Vector3d w_desired =
      Q_WT * (kp_rotation_ * Q_TTr_.vec().array()).matrix();

  // optimization for the motion component of dq.
  const auto prog = std::make_unique<solvers::MathematicalProgram>();
  auto dq = prog->NewContinuousVariables(num_positions_);

  // joint velocity cost
  prog->AddQuadraticErrorCost(
      velocity_cost_weight_ / std::pow(control_period, 2) * dq_weight_,
      Eigen::VectorXd::Zero(num_positions_), dq);

  // Jf null space constraint
  prog->AddLinearEqualityConstraint(Jf, 0, dq);

  // tracking error costs
  prog->AddL2NormCost(Jm / control_period, x_dot_desired, dq);
  prog->AddL2NormCost(J_rotation / control_period, w_desired, dq);

  solver_.Solve(*prog, {}, {}, prog_result_.get());

  if (!prog_result_->is_success()) {
    throw std::runtime_error("Controller QP cannot be solved.");
  }
  const Eigen::VectorXd dq_motion = prog_result_->GetSolution(dq);

  // calculate dq_force
  f_contact_ref_ = (1 - f_contact_growth_rate_) * f_contact_ref_ +
                   f_contact_growth_rate_ * f_contact_desired_;

  ContactInfo contact_info;
  contact_info.num_contacts = 1;
  contact_info.contact_link_idx.push_back(7);
  contact_info.positions.push_back(p_ToQ_T);
  const Eigen::Vector3d f_contact_C =
      R_WC.transpose() * contact_force_estimator_->UpdateContactForce(
                             contact_info, q, tau_external);

  // TODO: it's assumed here that exactly one of the three axes of C is force
  // controlled.
  const double f_contact = f_contact_C[force_axes[0]];

  double f_contact_cmd = f_contact_ref_;

  if (std::abs(f_contact_ref_ - f_contact_desired_) < 1) {
    // update integrator states.
    const double ki = 5;
    f_integrator_state_ += ki * (f_contact_ref_ - f_contact) * control_period;
//    std::cout << t << " " << f_contact_ref_ << " " << f_contact << std::endl;
    // Anti-windup.
    if (f_integrator_state_ > 5) {
      f_integrator_state_ = 5;
    } else if (f_integrator_state_ < -5) {
      f_integrator_state_ = -5;
    }

    f_contact_cmd += f_integrator_state_;
  }

  const Eigen::VectorXd dq_force =
      (-Jf.transpose().array() / joint_stiffness_ * f_contact_cmd).matrix();

  Eigen::VectorXd dq_all = dq_motion + dq_force;

  // saturation
  const double dq_limit = 10;
  ClipEigenVector(&dq_all, -dq_limit, dq_limit);

  *q_cmd = q + dq_all;
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
