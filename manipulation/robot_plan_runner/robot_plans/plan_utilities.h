#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

multibody::ModelInstanceIndex SetupIiwaControllerPlant(
    multibody::MultibodyPlant<double>* const plant);

void SetSmallValuesToZero(Eigen::VectorXd* const v_ptr, double tolerance);

template <class T>
void ClipEigenVector(T* const v, double min, double max);

class LowPassFilter {
 public:
  LowPassFilter(int dimension, double h, double w_cutoff);
  void Update(const Eigen::Ref<const Eigen::VectorXd>& u);
  void Update(const std::vector<double>& u);
  void reset_state();
  const Eigen::VectorXd& get_current_x() const;

 private:
  const double a_;
  const int n_;
  bool has_valid_state_{false};
  Eigen::VectorXd x_;
};

template <class T>
class FirstOrderSystem {
 public:
  FirstOrderSystem(const T& x_start, const T& x_end, double time_constant)
      : x_start_(x_start), x_end_(x_end), T_(time_constant){};

  T value(double t) const {
    return x_start_ + (x_end_ - x_start_) * (1 - std::exp(-t / T_));
  };

  double get_time_constant() { return T_; };

 private:
  const T x_start_;
  const T x_end_;
  const double T_;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
