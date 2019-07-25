
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

/*
 * This is not as much a test as it is an example of using
 * PlanRunnerHardwareInterface. To see what it does to robots, it should be run
 * together with //examples/manipulation_station:mock_station_simulation
 */
int run_plan() {
  // create plan
  // plan1 goes to the starting pose of the contact-aware plan.
  PlanData plan0;

  Eigen::VectorXd t_knots0(2);
  t_knots0 << 0, 1;

  Eigen::MatrixXd q_knots(7, 2);
  q_knots.col(0) << -0.5095, 1.1356, -0.0800, -1.4893, -0.3389, 1.2274, -0.3360;
  q_knots.col(1) = q_knots.col(0);

  auto qtraj = trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      t_knots0, q_knots);

  plan0.plan_type = PlanType::kJointSpacePlan;
  plan0.joint_traj = qtraj;

  // plan2 runs contact-aware plan.
  PlanData plan1;
  plan1.plan_type = PlanType::kTaskSpacePlanContact;

  PlanData::EeData ee_data;
  ee_data.p_ToQ_T.setZero();

  Eigen::Vector3d t_knots2(0, 8, 16);

  Eigen::MatrixXd xyz_knots(3, 3);
  xyz_knots.col(0) << 0, 0, 0;
  xyz_knots.col(2) << 0, -0.25, -0.15;
  xyz_knots.col(1) = (xyz_knots.col(0) + xyz_knots.col(2)) / 2;

  ee_data.ee_xyz_traj =
      trajectories::PiecewisePolynomial<double>::Cubic(
          t_knots2, xyz_knots, Eigen::VectorXd::Zero(3),
          Eigen::VectorXd::Zero(3));
  ee_data.ee_xyz_dot_traj = ee_data.ee_xyz_traj.derivative(1);

  auto Q_WT = math::RollPitchYawd(0, M_PI * 1.25, 0).ToQuaternion();

  vector<double> t_knots_v{0, 4, 8};
  vector<Eigen::Quaterniond> quaternions{Q_WT, Q_WT, Q_WT};

  ee_data.ee_quat_traj =
      trajectories::PiecewiseQuaternionSlerp<double>(t_knots_v, quaternions);

  plan1.ee_data = ee_data;
  vector<PlanData> plan_list{plan0, plan1};

  // Construct plan runner hardware interface.
  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list);

  // Run simulation.
  plan_runner.Run();

  return 0;
};

}  // namespace
}  // namespace drake

int main() { return drake::run_plan(); };