
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
    : PlanBase(PlanType::kHybridForcePositionPlan, 7),
      plant_(std::make_unique<multibody::MultibodyPlant<double>>()),
      kp_translation_(Eigen::Array3d(150, 150, 150)),
      kp_rotation_(Eigen::Array3d(50, 50, 50)),
      velocity_cost_weight_(0.01),
      f_contact_growth_rate_(0.005 / (1 + 0.005)),
      f_contact_desired_(10),
      f_contact_ref_(0) {
//      f_integrator_state_(0) {
  DRAKE_THROW_UNLESS(solver_.available());
  robot_model_ = SetupIiwaControllerPlant(plant_.get());
  plant_context_ = plant_->CreateDefaultContext();
  task_frame_idx_ = plant_->GetFrameByName("iiwa_link_7").index();
  Jv_WTq_.resize(6, num_positions_);

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

  // Update forward kinematics.
  const auto& task_def = plan_data.hybrid_task_definition.value();
  const auto& p_ToP_T = task_def.p_ToP_T;
  const auto& p_CoPr_C = task_def.p_CoPr_C;
  const auto& Q_CTr = task_def.Q_CTr;

  const auto X_WT =
      plant_->CalcRelativeTransform(*plant_context_, plant_->world_frame(),
                                    plant_->get_frame(task_frame_idx_));
  plant_->CalcFrameGeometricJacobianExpressedInWorld(
      *plant_context_, plant_->get_frame(task_frame_idx_), p_ToP_T, &Jv_WTq_);

  // Q_CT
  const auto Q_WT = X_WT.rotation().ToQuaternion();
  const auto Q_CW = task_def.Q_WC_traj.orientation(t).inverse();
  const auto Q_CT = Q_CW * Q_WT;

  // p_CoP_C
  const auto p_WoP_W = X_WT * p_ToP_T;
  const auto p_WoCo_W = task_def.p_WoCo_W_traj.value(t);
  const auto p_CoP_C = Q_CW * (p_WoP_W - p_WoCo_W);

  // Update position error.
  const auto p_PPr_C = p_CoPr_C - p_CoP_C;

  // Update orientation error.
  const auto Q_TTr = Q_CT.inverse() * Q_CTr;

  // Calculate translational velocity in C.
  const Vector3d v_CoPd_C = (kp_translation_ * p_PPr_C.array()).matrix();

  // Calculate angular velocity in C.
  const Vector3d w_CTd_C = (kp_rotation_ * Q_TTr.vec().array()).matrix();

  VectorXd V_C_TP_C_des(6);
  V_C_TP_C_des.head(3) = w_CTd_C;
  V_C_TP_C_des.tail(3) = v_CoPd_C;

  // Joacbians
  MatrixXd Jc(6, num_positions_);
  const auto R_CW = Q_CW.toRotationMatrix();
  Jc.topRows(3) = R_CW * Jv_WTq_.topRows(3);
  Jc.bottomRows(3) = R_CW * Jv_WTq_.bottomRows(3);


  // optimization for the motion component of dq.
  const auto prog = std::make_unique<solvers::MathematicalProgram>();
  auto dq = prog->NewContinuousVariables(num_positions_);

  // joint velocity cost
  prog->AddQuadraticErrorCost(
      velocity_cost_weight_ / std::pow(control_period, 2) * dq_weight_,
      Eigen::VectorXd::Zero(num_positions_), dq);

  // Jf null space constraint
//  prog->AddLinearEqualityConstraint(Jf, 0, dq);

  // tracking error costs
  prog->AddL2NormCost(Jc / control_period, V_C_TP_C_des, dq);
//  prog->AddL2NormCost(J_rotation / control_period, w_desired, dq);

  solver_.Solve(*prog, {}, {}, prog_result_.get());

  if (!prog_result_->is_success()) {
    throw std::runtime_error("Controller QP cannot be solved.");
  }
  const Eigen::VectorXd dq_motion = prog_result_->GetSolution(dq);

//  // calculate dq_force
  f_contact_ref_ = (1 - f_contact_growth_rate_) * f_contact_ref_ +
                   f_contact_growth_rate_ * f_contact_desired_;
//
//  ContactInfo contact_info;
//  contact_info.num_contacts = 1;
//  contact_info.contact_link_idx.push_back(7);
//  contact_info.positions.push_back(p_ToP_T);
//  const Eigen::Vector3d f_contact_C =
//      R_WC.transpose() * contact_force_estimator_->UpdateContactForce(
//                             contact_info, q, tau_external);
//
//  // TODO: it's assumed here that exactly one of the three axes of C is force
//  // controlled.
//  const double f_contact = f_contact_C[force_axes[0]];
//
//  double f_contact_cmd = f_contact_ref_;
//
//  if (std::abs(f_contact_ref_ - f_contact_desired_) < 1) {
//    // update integrator states.
//    const double ki = 5;
//    f_integrator_state_ += ki * (f_contact_ref_ - f_contact) * control_period;
////    std::cout << t << " " << f_contact_ref_ << " " << f_contact << std::endl;
//    // Anti-windup.
//    if (f_integrator_state_ > 5) {
//      f_integrator_state_ = 5;
//    } else if (f_integrator_state_ < -5) {
//      f_integrator_state_ = -5;
//    }
//
//    f_contact_cmd += f_integrator_state_;
//  }
//
//  const Eigen::VectorXd dq_force =
//      (-Jf.transpose().array() / joint_stiffness_ * f_contact_cmd).matrix();

  Eigen::VectorXd dq_all = dq_motion;

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
