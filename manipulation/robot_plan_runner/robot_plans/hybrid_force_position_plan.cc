
#include "drake/manipulation/robot_plan_runner/robot_plans/hybrid_force_position_plan.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_utilities.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RotationMatrixd;
using std::cout;
using std::endl;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

HybridForcePositionPlan::HybridForcePositionPlan()
    : PlanBase(PlanType::kHybridForcePositionPlan, 7),
      kp_translation_(Eigen::Array3d(150, 150, 150)),
      kp_rotation_(Eigen::Array3d(100, 100, 100)),
      velocity_cost_weight_(0.01) {
  DRAKE_THROW_UNLESS(solver_.available());

  plant_ = std::make_unique<multibody::MultibodyPlant<double>>();
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
  f_contact_ref_ = std::make_unique<FirstOrderSystem<double>>(0, 10, 0.8);
  f_integrator_state_ = 0;

  v_translation_norm_limit_ =
      std::make_unique<FirstOrderSystem<double>>(0.2, 1.0, 0.05);
}

Eigen::MatrixXd HybridForcePositionPlan::CalcSelectorMatrix(
    const std::vector<unsigned int>& axes) const {
  MatrixXd S;
  int n = axes.size();
  S.resize(n, 6);
  S.setZero();
  for (int i = 0; i < n; i++) {
    S(i, axes[i]) = 1;
  }
  return S;
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

//  cout << endl << "t: " << t << endl;
//  cout << "p_ToP_T\n" << p_ToP_T << endl;
//  cout << "p_CoPr_C\n" << p_CoPr_C << endl;
//  cout << "Q_CTr\n" << Q_CTr.toRotationMatrix() << endl;

  const auto X_WT =
      plant_->CalcRelativeTransform(*plant_context_, plant_->world_frame(),
                                    plant_->get_frame(task_frame_idx_));

  plant_->CalcJacobianSpatialVelocity(
      *plant_context_, multibody::JacobianWrtVariable::kQDot,
      plant_->get_frame(task_frame_idx_), p_ToP_T,
      plant_->world_frame(), plant_->world_frame(), &Jv_WTq_);

  // Q_CT
  const auto Q_WT = X_WT.rotation().ToQuaternion();
  const auto Q_CW = task_def.Q_WC_traj.orientation(t).inverse();
  const auto R_CW = Q_CW.toRotationMatrix();
  const auto Q_CT = Q_CW * Q_WT;

//  cout << "R_CW\n" << R_CW << endl;

  // p_CoP_C
  const auto p_WoP_W = X_WT * p_ToP_T;
  const auto p_WoCo_W = task_def.p_WoCo_W_traj.value(t);
  const auto p_CoP_C = Q_CW * (p_WoP_W - p_WoCo_W);

//  cout << "p_WoP_W\n" << p_WoP_W << endl;
//  cout << "p_WoCo_W\n" << p_WoCo_W << endl;
//  cout << "p_CoP_C\n" << p_CoP_C << endl;

  // Update position error.
  const auto p_PPr_C = p_CoPr_C - p_CoP_C;

//  cout << "p_PPr_C\n" << p_PPr_C << endl;

  // Update orientation error.
  const auto Q_TTr = RotationMatrixd(Q_CT.inverse() * Q_CTr).ToQuaternion();
//  cout << "Q_TTr\n" << Q_TTr.w() << endl << Q_TTr.vec() << endl;

  // Calculate translational velocity in C.
  Vector3d v_CoPd_C = kp_translation_ * p_PPr_C.array();
  const double v_norm = v_CoPd_C.norm();
  const double v_norm_limit = v_translation_norm_limit_->value(t);
  if (v_norm > v_norm_limit) {
    v_CoPd_C *= v_norm_limit / v_norm;
  }

  // Calculate angular velocity in C.
  const Vector3d w_CTd_C = Q_CT * (kp_rotation_ * Q_TTr.vec().array()).matrix();

  Vector6d V_C_TP_C_des;
  V_C_TP_C_des.head(3) = w_CTd_C;
  V_C_TP_C_des.tail(3) = v_CoPd_C;

//  cout << "V_C_TP_C_des\n" << V_C_TP_C_des << endl;

  Vector6d V_ff_c;
  if (t > plan_data.get_duration()) {
    V_ff_c.setZero();
  } else {
    V_ff_c.head(3) = R_CW * task_def.Q_WC_traj.angular_velocity(t);
    V_ff_c.tail(3) = R_CW * task_def.p_WoCo_W_traj.derivative(1).value(t);
  }

//  cout << "V_ff_c\n" << V_ff_c << endl;
//  cout << "v_co_ff\n" << task_def.p_WoCo_W_traj.derivative(1).value(t) << endl;

  // Joacbians
  MatrixXd Jc(6, num_positions_);
  Jc.topRows(3) = R_CW * Jv_WTq_.topRows(3);
  Jc.bottomRows(3) = R_CW * Jv_WTq_.bottomRows(3);

//  cout << "Jc\n" << Jc << endl;

  // Selector matrices
  const auto Sm = this->CalcSelectorMatrix(task_def.motion_controlled_axes);
//  cout << "S_m\n" << Sm << endl;

  // optimization for the motion component of dq.
  const auto prog = std::make_unique<solvers::MathematicalProgram>();
  auto dq = prog->NewContinuousVariables(num_positions_);

  // joint velocity cost
  prog->AddQuadraticErrorCost(
      velocity_cost_weight_ / std::pow(control_period, 2) * dq_weight_,
      Eigen::VectorXd::Zero(num_positions_), dq);

  // tracking error costs
  prog->AddL2NormCost(Sm * Jc / control_period, Sm * (V_C_TP_C_des + V_ff_c),
                      dq);

  // control contact force.
  const unsigned int nf = task_def.force_controlled_axes.size();
  VectorXd dq_force(num_positions_);
  dq_force.setZero();
  if (nf > 0) {
    // TODO: it's assumed here that exactly one of the three axes of C is
    // force controlled.
    DRAKE_THROW_UNLESS(nf == 1);
    const auto Sf = this->CalcSelectorMatrix(task_def.force_controlled_axes);
    const auto Jf = Sf * Jc;

//    cout << "Jf\n" << Jf << endl;
//    cout << "Sf\n" << Sf << endl;

    // Jf null space constraint
    prog->AddLinearEqualityConstraint(Jf, 0, dq);

    ContactInfo contact_info;
    contact_info.num_contacts = 1;
    contact_info.contact_link_idx.push_back(7);
    contact_info.positions.push_back(p_ToP_T);
    const Eigen::Vector3d F_C =
        R_CW * contact_force_estimator_->UpdateContactForce(contact_info, q,
                                                            tau_external);

    const double f_contact = F_C[task_def.force_controlled_axes[0] - 3];

    const double f_contact_ref = f_contact_ref_->value(t);
    double f_contact_cmd = f_contact_ref;

    if (t / f_contact_ref_->get_time_constant() > 3) {
      // update integrator states.
      f_integrator_state_ += 5 * (f_contact_ref - f_contact) * control_period;

      // Anti-windup.
      if (f_integrator_state_ > 5) {
        f_integrator_state_ = 5;
      } else if (f_integrator_state_ < -5) {
        f_integrator_state_ = -5;
      }

      f_contact_cmd += f_integrator_state_;
    }

//    cout << "f_contact r, cmd" << f_contact_ref << " " << f_contact_cmd <<
//    endl;

    dq_force = -Jf.transpose().array() / joint_stiffness_ * f_contact_cmd;
  }

  solver_.Solve(*prog, {}, {}, prog_result_.get());

  if (!prog_result_->is_success()) {
    throw std::runtime_error("Controller QP cannot be solved.");
  }
  const Eigen::VectorXd dq_motion = prog_result_->GetSolution(dq);

  *q_cmd = q + dq_motion + dq_force;
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
