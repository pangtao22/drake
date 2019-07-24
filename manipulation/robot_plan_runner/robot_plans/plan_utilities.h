#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

multibody::ModelInstanceIndex SetupIiwaControllerPlant(
    multibody::MultibodyPlant<double>* const plant);

class ContactForceEstimator {
 public:
  explicit ContactForceEstimator(double w_cutoff);

  Eigen::Vector3d UpdateContactForce(
      const Eigen::Ref<const Eigen::Vector3d>& pC_C,
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const Eigen::VectorXd>& tau_external,
      double control_period) const;

  Eigen::RowVectorXd CalcContactJacobian();

 private:
  Eigen::Vector3d EstimateContactForce(
      const Eigen::Ref<const Eigen::Vector3d>& pC_C,
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const Eigen::VectorXd>& tau_external) const;

  void LowPassFilterContactForce(const Eigen::Ref<const Eigen::Vector3d>& f_new,
                                 double h) const;

  std::unique_ptr<multibody::MultibodyPlant<double>> plant_;
  multibody::FrameIndex contact_frame_idx_;
  multibody::ModelInstanceIndex robot_model_;
  mutable std::unique_ptr<systems::Context<double>> plant_context_;
  int num_positions_{0};

  // Jacobian of contact point (c) relative to world frame
  // expressed in world frame, (3 * num_positions_).
  mutable Eigen::MatrixXd Jv_WCc_;

  // filtered contact force
  mutable std::unique_ptr<Eigen::Vector3d> f_filtered_;

  // cutoff frequency of the low-pass filter on contact force, in rad/s.
  const double w_cutoff_;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
