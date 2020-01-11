
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/task_space_plan_contact.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_utilities.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using std::cout;
using std::endl;

TaskSpacePlanContact::TaskSpacePlanContact()
    : TaskSpacePlan(), velocity_cost_weight_(0.1) {
  DRAKE_THROW_UNLESS(solver_.available());
  this->set_plan_type(PlanType::kTaskSpacePlanContact);

  contact_force_estimator_ =
      std::make_unique<ContactForceEstimator>(0.005, 0.5 * 2 * M_PI);

  joint_stiffness_.resize(num_positions_);
  joint_stiffness_ << 800, 600, 600, 600, 400, 200, 200;

  dq_weight_.resize(num_positions_, num_positions_);
  dq_weight_.setZero();
  dq_weight_.diagonal() << 5, 4, 3, 3, 2, 2, 1;

  prog_result_ = std::make_unique<solvers::MathematicalProgramResult>();
}

void TaskSpacePlanContact::UpdatePositionErrorUsingTargetPoint(
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

void TaskSpacePlanContact::Step(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
    double control_period, double t, const PlanData& plan_data,
    const robot_plans::ContactInfo&, EigenPtr<Eigen::VectorXd> q_cmd,
    EigenPtr<Eigen::VectorXd> tau_cmd) const {
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

  plant_->CalcJacobianSpatialVelocity(
      *plant_context_, multibody::JacobianWrtVariable::kQDot,
      plant_->get_frame(task_frame_idx_), p_ToQ_T,
      plant_->world_frame(), plant_->world_frame(), &Jv_WTq_);

  // Update errors.
  this->UpdatePositionError(t, plan_data, p_WoQ_W);
  this->UpdateOrientationError(t, plan_data, Q_WT);

  // Update x_dot_desired.
  x_dot_desired_.tail(3) = kp_translation_ * err_xyz_.array();
  x_dot_desired_.head(3) =
      Q_WT * (kp_rotation_ * Q_TTr_.vec().array()).matrix();

  // Estimate contact force, assuming the contact is at the center of the
  // sphere.
  const Eigen::Vector3d pC_T(0, 0, 0.075);
  ContactInfo contact_info;
  contact_info.num_contacts = 1;
  contact_info.contact_link_idx.push_back(7);
  contact_info.positions.push_back(pC_T);
  const Eigen::Vector3d f_contact =
      contact_force_estimator_->UpdateContactForce(contact_info, q,
                                                   tau_external);

  // MahtematicalProgram-related declarations.
  const auto prog = std::make_unique<solvers::MathematicalProgram>();
  auto dq = prog->NewContinuousVariables(num_positions_);

  // alias for task Jacobian.
  const Eigen::MatrixXd& Jt = Jv_WTq_;

  // joint velocity cost
  prog->AddQuadraticErrorCost(
      velocity_cost_weight_ / std::pow(control_period, 2) * dq_weight_,
      Eigen::VectorXd::Zero(num_positions_), dq);

  // Deal with contact.
  const double f_norm_threshold = 10;
  const double f_norm = f_contact.norm();
  Eigen::VectorXd dq_force(num_positions_);
  dq_force.setZero();
  if (f_norm > f_norm_threshold) {
    const Eigen::RowVectorXd J_nc =
        contact_force_estimator_->CalcContactJacobian();

    const Eigen::VectorXd J_nc_pinv =
        J_nc.transpose() / std::pow(J_nc.norm(), 2);

    const double f_desired = f_norm_threshold * 1.5;
    prog->AddLinearEqualityConstraint(
        (J_nc_pinv.array() * joint_stiffness_).matrix().transpose(), -f_desired,
        dq);

    prog->AddLinearConstraint(J_nc / control_period,
                              -std::numeric_limits<double>::infinity(), 0, dq);
  }
  // tracking error cost
  prog->AddL2NormCost(Jt / control_period, x_dot_desired_, dq);

  solver_.Solve(*prog, {}, {}, prog_result_.get());

  if (!prog_result_->is_success()) {
    throw std::runtime_error("Controller QP cannot be solved.");
  }
  auto dq_value = prog_result_->GetSolution(dq);

  // saturation
  const double dq_limit = 10;
  ClipEigenVector(&dq_value, -dq_limit, dq_limit);

  *q_cmd = q + dq_value;
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
