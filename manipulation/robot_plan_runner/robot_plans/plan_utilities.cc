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
 *  w_cutoff: cutoff frequency of the low-pass filter on contact force, in
 *  rad/s.
 *  h: controller time step in seconds.
 */
LowPassFilter::LowPassFilter(int dimension, double h, double w_cutoff):
    a_(h * w_cutoff / (1 + h * w_cutoff)), n_(dimension) {
  x_.resize(n_);
}

/*
 * x: state of the low-pass filter system.
 * x(k+1) = (1-a) * x(k) + a * u
 */
void LowPassFilter::Update(const Eigen::Ref<const Eigen::VectorXd>& u) {
  if(has_valid_state_) {
    x_ = (1 - a_) * x_ + a_ * u;
  } else {
    has_valid_state_ = true;
    x_ = u;
  }
}

void LowPassFilter::reset() {has_valid_state_ = false; }

Eigen::VectorXd LowPassFilter::get_current_x() {
  if(has_valid_state_) {
    return x_;
  } else {
    throw std::runtime_error("LowPassFilter does not have a valid state.");
  }
}



}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
