#include "drake/manipulation/robot_plan_runner/robot_plans/joint_space_plan_contact.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using std::cout;
using std::endl;

JointSpacePlanContact::JointSpacePlanContact(int num_positions)
    : PlanBase(PlanType::kJointSpacePlanContact, num_positions),
      positive_v_count_(0) {
  DRAKE_THROW_UNLESS(solver_.available());

  contact_force_estimator_ =
      std::make_unique<ContactForceEstimator>(0.005, 0.5 * 2 * M_PI);

  joint_stiffness_.resize(num_positions_);
  joint_stiffness_ << 800, 600, 600, 600, 400, 200, 200;

  dq_weight_.resize(num_positions_, num_positions_);
  dq_weight_.setZero();
  dq_weight_.diagonal() << 5, 4, 3, 3, 2, 2, 1;

  prog_result_ = std::make_unique<solvers::MathematicalProgramResult>();
}

void JointSpacePlanContact::Step(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>&,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
    double control_period, double t, const PlanData& plan_data,
    const robot_plans::ContactInfo& contact_info,
    EigenPtr<Eigen::VectorXd> q_cmd, EigenPtr<Eigen::VectorXd> tau_cmd) const {
  DRAKE_THROW_UNLESS(plan_data.plan_type == plan_type_);

  // Evaluate q reference from plan_data.
  const auto& q_traj = plan_data.joint_traj.value();
  Eigen::VectorXd dq_ref = q_traj.value(t) - q;

  // MahtematicalProgram-related declarations.
  const auto prog = std::make_unique<solvers::MathematicalProgram>();
  auto dq = prog->NewContinuousVariables(num_positions_);

  // Estimate contact force, assuming the contact is at the center of the
  // sphere.
  //  const Eigen::Vector3d pC_T(0, 0, 0.075);
  //  ContactInfo contact_info;
  //  contact_info.num_contacts = 1;
  //  contact_info.contact_link_idx.push_back(7);
  //  contact_info.positions.push_back(pC_T);

  const Eigen::Vector3d f_contact =
      contact_force_estimator_->UpdateContactForce(contact_info, q,
                                                   tau_external);

  const double f_norm_threshold = 10;
  const double f_norm = f_contact.norm();

  if (f_norm > f_norm_threshold) {
    const Eigen::RowVectorXd J_u =
        contact_force_estimator_->CalcContactJacobian();
    const double dx_along_f = (J_u * dq_ref)[0];

    // saturate the norm of dq_ref.
    const double dq_ref_norm = dq_ref.norm();
    const double dq_ref_norm_threshold = 0.04;
    if (dq_ref_norm > dq_ref_norm_threshold) {
      dq_ref *= dq_ref_norm_threshold / dq_ref_norm;
    }

    if (dx_along_f > 0) {
      positive_v_count_++;
    } else {
      positive_v_count_ = 0;
    }

    if (positive_v_count_ >= 5 && !desired_contact_force_) {
      desired_contact_force_ = std::make_unique<FirstOrderSystem<double>>(
          f_norm_threshold * 1.5, f_norm_threshold * 0.8, 0.12);
      t_separation_ = t;
      cout << "separation starts!" << endl;
    }

    double f_desired = f_norm_threshold * 1.5;
    if (desired_contact_force_) {
      f_desired = desired_contact_force_->value(t - t_separation_);
      cout << t << " " << (t - t_separation_) << " " << f_desired << endl;
    }

    Eigen::VectorXd J_nc_pinv = J_u.transpose() / std::pow(J_u.norm(), 2);
    SetSmallValuesToZero(&J_nc_pinv, 1e-13);

    prog->AddLinearEqualityConstraint(
        (J_nc_pinv.array() * joint_stiffness_).matrix().transpose(), -f_desired,
        dq);
    prog->AddLinearConstraint(J_u / control_period,
                              -std::numeric_limits<double>::infinity(), 0, dq);
  }

  // Error on tracking error
  prog->AddQuadraticErrorCost(
      Eigen::MatrixXd::Identity(num_positions_, num_positions_) /
          std::pow(control_period, 2),
      dq_ref, dq);
  solver_.Solve(*prog, {}, {}, prog_result_.get());

  if (!prog_result_->is_success()) {
    throw std::runtime_error("Controller QP cannot be solved.");
  }
  auto dq_value = prog_result_->GetSolution(dq);

  // saturation
  //  const double dq_limit = 1;
  //  ClipEigenVector(&dq_value, -dq_limit, dq_limit);

  *q_cmd = q + dq_value;
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake