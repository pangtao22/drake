
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"

namespace drake {
namespace {

using manipulation::robot_plan_runner::robot_plans::PlanData;
using manipulation::robot_plan_runner::robot_plans::PlanType;
using std::cout;
using std::endl;
using std::vector;

int run_plan() {
  // create plan
  // plan1 goes to the starting pose of the contact-aware plan.
  PlanData plan0;

  Eigen::VectorXd t_knots0(2);
  t_knots0 << 0, 1;

  Eigen::MatrixXd q_knots0(7, 2);
  q_knots0.col(0) << 0, 0, 0, -1.75, 0, 0.9, 0;
  q_knots0.col(1) = q_knots0.col(0);

  plan0.plan_type = PlanType::kJointSpacePlan;
  plan0.joint_traj = trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      t_knots0, q_knots0);


  // plan1 commands a step in one of the joints.
  PlanData plan1;

  Eigen::VectorXd t_knots1(2);
  t_knots1 << 0, 2;
  Eigen::MatrixXd q_knots1(7, 2);
  q_knots1 = q_knots0;
  q_knots1(6, 0) += 0.005;

  plan1.plan_type = PlanType::kJointSpacePlan;
  plan1.joint_traj = trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      t_knots1, q_knots1);

  vector<PlanData> plan_list{plan0, plan1};

  // Construct plan runner hardware interface.
  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list);

  // Run simulation.
  plan_runner.Run();

  return 0;
}

}  // namespace
}  // namespace drake

int main() { return drake::run_plan(); }