#include "drake/manipulation/robot_plan_runner/robot_plans/plan_utilities.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

multibody::ModelInstanceIndex SetupIiwaControllerPlant(
    multibody::MultibodyPlant<double>* const plant) {
  multibody::Parser parser(plant);

  const char kIiwaSdf[] =
      "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_no_collision.sdf";

  std::string iiwa_sdf_path = FindResourceOrThrow(kIiwaSdf);
  parser.AddModelFromFile(iiwa_sdf_path, "iiwa7");
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
  plant->Finalize();

  return plant->GetModelInstanceByName("iiwa7");
}

/*
 * w_cutoff: cut off frequency in rad/s, recommended value 0.5 * 2 * M_PI.
 */
ContactForceEstimator::ContactForceEstimator(double w_cutoff)
    : plant_(std::make_unique<multibody::MultibodyPlant<double>>()),
      w_cutoff_(w_cutoff) {
  robot_model_ = SetupIiwaControllerPlant(plant_.get());
  plant_context_ = plant_->CreateDefaultContext();

  // TODO: contact frame should not be hard-coded. It should come from contact
  // particle filter,
  contact_frame_idx_ = plant_->GetFrameByName("iiwa_link_7").index();
  num_positions_ = plant_->num_positions();

  // contact Jacobian.
  Jv_WCc_.resize(3, num_positions_);
}

/*
 * pC_C: coordinate of the contact point in the frame of the body under
 *   contact (C). For now it is assumed to be the same as the task frame.
 * tau_external: external joint torque estimated by the robot.
 */
Eigen::Vector3d ContactForceEstimator::EstimateContactForce(
    const Eigen::Ref<const Eigen::Vector3d>& pC_C,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external) const {
  // Update q in plant_context.
  plant_->SetPositions(plant_context_.get(), robot_model_, q);

  // updates contact jacobian.
  Eigen::Vector3d p_WC;
  plant_->CalcPointsGeometricJacobianExpressedInWorld(
      *plant_context_, plant_->get_frame(contact_frame_idx_), pC_C, &p_WC,
      &Jv_WCc_);

  // least square solve Jv_WCc.T.dot(f) = tau_external.
  auto svd =
      Jv_WCc_.transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  return svd.solve(tau_external);
}

void ContactForceEstimator::LowPassFilterContactForce(
    const Eigen::Ref<const Eigen::Vector3d>& f_new, double h) const {
  const double a = h * w_cutoff_ / (1 + h * w_cutoff_);
  if (f_filtered_) {
    *f_filtered_ = (1 - a) * (*f_filtered_) + a * f_new;
  } else {
    f_filtered_ = std::make_unique<Eigen::Vector3d>(f_new);
  }
}

Eigen::Vector3d ContactForceEstimator::UpdateContactForce(
    const Eigen::Ref<const Eigen::Vector3d>& pC_C,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
    double control_period) const {
  const Eigen::Vector3d f_contact =
      this->EstimateContactForce(pC_C, q, tau_external);
  this->LowPassFilterContactForce(f_contact, control_period);

  return *f_filtered_;
}

Eigen::RowVectorXd ContactForceEstimator::CalcContactJacobian() {
  auto f_norm = (*f_filtered_).norm();
  Eigen::Vector3d contact_normal = *f_filtered_ / f_norm;
  return contact_normal.transpose() * Jv_WCc_;
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
