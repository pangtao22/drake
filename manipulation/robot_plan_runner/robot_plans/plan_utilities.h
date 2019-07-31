#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

multibody::ModelInstanceIndex SetupIiwaControllerPlant(
    multibody::MultibodyPlant<double>* const plant);


class LowPassFilter {
 public:
  LowPassFilter(int dimension, double h, double w_cutoff);
  void Update(const Eigen::Ref<const Eigen::VectorXd>& u);
  void reset();
  Eigen::VectorXd get_current_x();
 private:
  const double a_;
  const int n_;
  bool has_valid_state_{false};
  Eigen::VectorXd x_;
};


}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
