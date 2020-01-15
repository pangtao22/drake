#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/manipulation/robot_plan_runner/robot_plans/plan_utilities.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

struct ContactInfo {
  ContactInfo() = default;
  int num_contacts{0};
  std::vector<int> contact_link_idx{};
  std::vector<Eigen::Vector3d> positions{};
};

class ContactForceEstimator {
 public:
  explicit ContactForceEstimator(double h, double w_cutoff);

  Eigen::Vector3d UpdateContactForce(
      const ContactInfo& contact_info,
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const Eigen::VectorXd>& tau_external) const;

  Eigen::RowVectorXd CalcContactJacobian();

 private:
  Eigen::Vector3d EstimateContactForce(
      const ContactInfo& contact_info,
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const Eigen::VectorXd>& tau_external) const;

  std::unique_ptr<multibody::MultibodyPlant<double>> plant_;
  std::vector<multibody::FrameIndex> robot_frames_idx_{};
  multibody::ModelInstanceIndex robot_model_;
  mutable std::unique_ptr<systems::Context<double>> plant_context_;
  int num_positions_{0};

  // Jacobian of contact point (c) relative to world frame
  // expressed in world frame, (3 * num_positions_).
  // Contact kinematics. contact frame: L; C: contact point.
  mutable Eigen::MatrixXd Jv_WLc_W_;

  // filtered contact force
  std::unique_ptr<LowPassFilter> lpf_;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
