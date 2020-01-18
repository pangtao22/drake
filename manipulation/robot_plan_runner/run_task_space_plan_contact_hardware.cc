
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

int run_plan() {
  Eigen::VectorXd q0(7);
  const double duration = 26;

  //  // EE pushing against table.
  //  q0 << -0.4532, 1.1335, -0.0479, -1.3982, -0.3034, 1.3395, -0.3055;
  //  auto Q_WT = math::RollPitchYawd(0, M_PI * 1.25, 0).ToQuaternion();

  //  // Link 6 making contact with table, the one in the 1st draft of the
  //  paper. q0 << -0.965894, 1.41052, 0.548241, -1.46713, -0.973198, -1.49595,
  //  -0.0991943; auto Q_WT = math::RollPitchYawd(0, M_PI / 3,
  //  0).ToQuaternion(); Eigen::Vector3d t_knots1(0, duration / 2, duration);
  //  Eigen::MatrixXd xyz_knots(3, 3);
  //  xyz_knots.col(0) << 0, 0, 0;
  //  xyz_knots.col(2) << 0, 0, -0.25;
  //  xyz_knots.col(1) = (xyz_knots.col(0) + xyz_knots.col(2)) / 2;

  // reaching into bin
  q0 << -1.48595109, 1.25825224, 0.7347296, -1.60280037, -1.15394412,
      0.77650079, -1.76714587;
  auto Q_WT = math::RollPitchYawd(M_PI, 0, -M_PI / 2).ToQuaternion();

  Eigen::VectorXd t_knots1(4);
  t_knots1 << 0, 10, 13, 23;
  Eigen::MatrixXd xyz_knots(3, 4);
  xyz_knots.col(0) << 0, 0, 0;
  xyz_knots.col(1) << 0, 0, -0.25;
  xyz_knots.col(2) << 0, 0, -0.25;
  xyz_knots.col(3) << 0, 0, 0;

  // create plan
  // plan0 goes to the starting pose of the contact-aware plan.
  PlanData plan0;

  Eigen::VectorXd t_knots0(2);
  t_knots0 << 0, 1;

  Eigen::MatrixXd q_knots(7, 2);
  q_knots.col(0) = q0;

  auto qtraj = trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      t_knots0, q_knots);

  plan0.plan_type = PlanType::kJointSpacePlan;
  plan0.joint_traj = qtraj;

  // plan1 runs contact-aware plan.
  PlanData plan1;
  plan1.plan_type = PlanType::kTaskSpacePlanContact;

  PlanData::EeData ee_data;
  ee_data.p_ToQ_T << 0, 0, 0.10;
  ee_data.ee_xyz_traj =
      trajectories::PiecewisePolynomial<double>::FirstOrderHold(t_knots1,
                                                                xyz_knots);
  ee_data.ee_xyz_dot_traj = ee_data.ee_xyz_traj.derivative(1);

  vector<double> t_knots_v{0, duration / 2, duration};
  vector<Eigen::Quaterniond> quaternions{Q_WT, Q_WT, Q_WT};
  ee_data.ee_quat_traj =
      trajectories::PiecewiseQuaternionSlerp<double>(t_knots_v, quaternions);

  plan1.ee_data = ee_data;
  vector<PlanData> plan_list{plan0, plan1};

  // Construct plan runner hardware interface.
  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list,
                                                                   true);
  plan_runner.SaveGraphvizStringToFile();

  // Run simulation.
  plan_runner.Run();

  return 0;
}

}  // namespace
}  // namespace drake

int main() { return drake::run_plan(); }