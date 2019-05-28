
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"

namespace drake {
namespace {

using manipulation::robot_plan_runner::robot_plans::PlanData;
using manipulation::robot_plan_runner::robot_plans::PlanType;
using std::cout;
using std::endl;

int do_main() {
  // create plan
  PlanData plan1;

  Eigen::VectorXd t_knots(3);
  t_knots << 0, 2, 4;

  Eigen::MatrixXd q_knots(7, 3);
  q_knots.col(0) << 0, 0, 0, -1.75, 0, 1.0, 0;
  q_knots.col(1) << 0.5, 0, 0, -1.75, 0, 1.0, 0;
  q_knots.col(2) << 1, 0, 0, -1.75, 0, 1.0, 0;

  auto qtraj = trajectories::PiecewisePolynomial<double>::Cubic(
      t_knots, q_knots, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));

  plan1.plan_type = PlanType::kJointSpacePlan;
  plan1.joint_traj = qtraj;
  std::vector<PlanData> plan_list{plan1};

  // Construct plan runner hardware interface.
  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list);

  // save diagram graphviz string.
  plan_runner.SaveGraphvizStringToFile();

  // Run simulation.
  plan_runner.Run();

  return 0;
};

}  // namespace
}  // namespace drake

int main() { return drake::do_main(); };