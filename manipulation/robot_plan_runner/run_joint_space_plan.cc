
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace {

using manipulation::robot_plan_runner::robot_plans::PlanData;
using manipulation::robot_plan_runner::robot_plans::PlanType;
using math::RollPitchYawd;
using std::cout;
using std::endl;
using std::vector;

int do_main() {
  PlanData plan0;
  plan0.plan_type = PlanType::kJointSpacePlan;

  Eigen::VectorXd t_knots0(2);
  t_knots0 << 0, 1;

  Eigen::MatrixXd q_knots(7, 2);
  q_knots.col(0) <<  -0.474, 0.952, 0.059, -1.754, -0.114, 0.438, -0.337;
  q_knots.col(1) = q_knots.col(0);

  auto qtraj = trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      t_knots0, q_knots);

  plan0.joint_traj = qtraj;


  vector<PlanData> plan_list{plan0};

  // Construct plan runner hardware interface.
  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list);

  // save diagram graphviz string.
  plan_runner.SaveGraphvizStringToFile();

  // Run simulation.
  plan_runner.Run();

  return 0;

}

}  // namespace
}  // namespace drake

int main() { return drake::do_main(); }