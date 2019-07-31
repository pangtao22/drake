
#include "drake/manipulation/robot_plan_runner/robot_plans/contact_force_estimator.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

/*
 * w_cutoff: cut off frequency in rad/s, recommended value 0.5 * 2 * M_PI.
 */
ContactForceEstimator::ContactForceEstimator(double h, double w_cutoff)
    : plant_(std::make_unique<multibody::MultibodyPlant<double>>()),
      lpf_(std::make_unique<LowPassFilter>(3, h, w_cutoff)) {
  robot_model_ = SetupIiwaControllerPlant(plant_.get());
  plant_context_ = plant_->CreateDefaultContext();

  num_positions_ = plant_->num_positions();

  for (int i = 0; i < plant_->num_bodies() - 1; i++) {
    robot_frames_idx_.push_back(
        plant_->GetFrameByName("iiwa_link_" + std::to_string(i)).index());
  }

  // contact Jacobian.
  Jv_WCc_.resize(3, num_positions_);
}

/*
 * pC_C: coordinate of the contact point in the frame of the body under
 *   contact (C). For now it is assumed to be the same as the task frame.
 * tau_external: external joint torque estimated by the robot.
 */
Eigen::Vector3d ContactForceEstimator::EstimateContactForce(
    const ContactInfo& contact_info, const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external) const {
  // Update q in plant_context.
  plant_->SetPositions(plant_context_.get(), robot_model_, q);

  // updates contact jacobian.
  const Eigen::Vector3d& pC_C = contact_info.positions[0];
  Eigen::Vector3d p_WC;
  plant_->CalcPointsGeometricJacobianExpressedInWorld(
      *plant_context_,
      plant_->get_frame(robot_frames_idx_[contact_info.contact_link_idx[0]]),
      pC_C, &p_WC, &Jv_WCc_);

  // least square solve Jv_WCc.T.dot(f) = tau_external.
  auto svd =
      Jv_WCc_.transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  return svd.solve(tau_external);
}

Eigen::Vector3d ContactForceEstimator::UpdateContactForce(
    const ContactInfo& contact_info, const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external) const {
  DRAKE_THROW_UNLESS(contact_info.num_contacts <= 1);

  Eigen::Vector3d f_contact_new;
  if(contact_info.num_contacts == 1) {
    f_contact_new = this->EstimateContactForce(contact_info, q, tau_external);
  } else {
    f_contact_new.setZero();
  }

  lpf_->Update(f_contact_new);
  return lpf_->get_current_x();
}

Eigen::RowVectorXd ContactForceEstimator::CalcContactJacobian() {
  Eigen::Vector3d f = lpf_->get_current_x();
  auto f_norm = f.norm();
  Eigen::Vector3d contact_normal = f / f_norm;
  return contact_normal.transpose() * Jv_WCc_;
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
