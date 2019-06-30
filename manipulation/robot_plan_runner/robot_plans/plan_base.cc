#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"


namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

void PlanBase::check_plan_type(const PlanData& plan_data) const {
  if (plan_data.plan_type != plan_type_) {
    throw std::runtime_error("Mismatch between Plan and PlanData.");
  }
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
