#include "drake/manipulation/robot_plan_runner/schunk_command_sender.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

SchunkCommandSender::SchunkCommandSender() {
  this->set_name("SchunkCommandSender");

  this->DeclareAbstractOutputPort("schunk_command",
                                  &SchunkCommandSender::CalcOutput);

  t_knots_ = {0, 9 + 8, 1e6};
  gripper_setpoints_ = {25, 5};
//  t_knots_ = {0, 9 + 5, 1e6};
//  gripper_setpoints_ = {2, 20};
  current_setpoint_index_ = 0;
  DRAKE_THROW_UNLESS(t_knots_.size() == gripper_setpoints_.size() + 1);
}

void SchunkCommandSender::CalcOutput(const systems::Context<double>& context,
                                     lcmt_schunk_wsg_command* output) const {
  const double t = context.get_time();
  lcmt_schunk_wsg_command& command = *output;

  if (t > t_knots_[current_setpoint_index_ + 1]) {
    current_setpoint_index_++;
  }

  command.utime = t * 1e6;
  command.force = 10;
  command.target_position_mm = gripper_setpoints_[current_setpoint_index_];
}

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
