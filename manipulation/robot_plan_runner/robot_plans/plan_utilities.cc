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

void SetSmallValuesToZero(Eigen::VectorXd* const v_ptr, double tolerance) {
  auto& v = *v_ptr;
  const int n = v.size();
  for (int i = 0; i < n; i++) {
    if (abs(v(i)) <= tolerance) {
      v(i) = 0;
    }
  }
}

template <class T>
void ClipEigenVector(T* const v_ptr, double min, double max) {
  auto& v = *v_ptr;
  const int n = v.size();

  for (int i = 0; i < n; i++) {
    if (v(i) > max) {
      v(i) = max;
    } else if (v(i) < min) {
      v(i) = min;
    }
  }
}

/*
 *  w_cutoff: cutoff frequency of the low-pass filter on contact force, in
 *  rad/s.
 *  h: controller time step in seconds.
 */
LowPassFilter::LowPassFilter(int dimension, double h, double w_cutoff)
    : a_(h * w_cutoff / (1 + h * w_cutoff)), n_(dimension) {
  x_.resize(n_);
}

/*
 * x: state of the low-pass filter system.
 * x(k+1) = (1-a) * x(k) + a * u
 */
void LowPassFilter::Update(const Eigen::Ref<const Eigen::VectorXd>& u) {
  DRAKE_THROW_UNLESS(u.size() == n_);
  if (has_valid_state_) {
    x_ = (1 - a_) * x_ + a_ * u;
  } else {
    has_valid_state_ = true;
    x_ = u;
  }
}

void LowPassFilter::Update(const std::vector<double>& u) {
  this->Update(Eigen::Map<const Eigen::VectorXd>(u.data(), u.size()));
}

void LowPassFilter::reset_state() { has_valid_state_ = false; }

const Eigen::VectorXd& LowPassFilter::get_current_x() const {
  if (has_valid_state_) {
    return x_;
  } else {
    throw std::runtime_error("LowPassFilter does not have a valid state.");
  }
}

template void ClipEigenVector(Eigen::VectorXd* const v_ptr, double min,
                              double max);

template void ClipEigenVector(Eigen::Vector3d* const v_ptr, double min,
                              double max);

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
