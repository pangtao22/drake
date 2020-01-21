#pragma once

#include <vector>

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

class SchunkCommandSender : public systems::LeafSystem<double> {
 public:
  SchunkCommandSender();

 private:
  void CalcOutput(const drake::systems::Context<double>&,
                  lcmt_schunk_wsg_command*) const;
  mutable std::vector<double> t_knots_;
  mutable std::vector<double> gripper_setpoints_;
  mutable int current_setpoint_index_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
