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

double PlanData::get_duration() const {
  if (joint_traj.has_value()) {
    return joint_traj.value().end_time();
  } else if (ee_data.has_value()) {
    // TODO(pangtao22): throw if durations of ee_xyz_traj and ee_quat_traj
    //  are different.
    return ee_data.value().ee_xyz_traj.end_time();
  } else if (hybrid_task_definition.has_value()) {
    return hybrid_task_definition.value().p_WoCo_W_traj.end_time();
  } else {
    throw std::runtime_error("invalid PlanData.");
  }
}

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
