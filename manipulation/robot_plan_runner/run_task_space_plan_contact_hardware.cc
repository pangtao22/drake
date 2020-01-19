
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_utilities.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace {

using manipulation::robot_plan_runner::robot_plans::PlanData;
using manipulation::robot_plan_runner::robot_plans::PlanType;
using math::RollPitchYawd;
using std::cout;
using std::endl;
using std::vector;
using trajectories::PiecewisePolynomial;

Eigen::VectorXd CalcInitialJointAngles() {
  auto plant = std::make_unique<multibody::MultibodyPlant<double>>(1e-3);
  manipulation::robot_plan_runner::robot_plans::SetupIiwaControllerPlant(
      plant.get());

  const int nq = 7;  // number of joints.
  const double theta_bound = 0.001;
  const double position_tolerance = 0.0001;

  const Eigen::Vector3d p_WQ_start(0.30, -0.38 - 0.058, 0);
  const double yaw_angle = -M_PI / 16;
  const auto R_WL6 =
      math::RollPitchYawd(0, -M_PI, yaw_angle).ToRotationMatrix();
  Eigen::VectorXd q_initial_guess(nq);
  q_initial_guess << -64.78, 84.36, 17.12, -69.87, -38.88, 33.77, -116.53;
  q_initial_guess *= M_PI / 180;  // convert to radians.
  const Eigen::Vector3d p_L7oQ_L7(0, 0, 0.1);

  const auto& l6_frame = plant->GetFrameByName("iiwa_link_6");
  const auto& l7_frame = plant->GetFrameByName("iiwa_link_7");

  auto context = plant->CreateDefaultContext();
  const auto X_WL6_0 =
      plant->CalcRelativeTransform(*context, plant->world_frame(), l6_frame);
  const auto X_L6W_0 = X_WL6_0.inverse();

  Eigen::VectorXd q0(nq);

  auto solver = std::make_unique<solvers::SnoptSolver>();
  auto result = std::make_unique<solvers::MathematicalProgramResult>();

  multibody::InverseKinematics ik(*plant);

  ik.AddOrientationConstraint(plant->world_frame(), R_WL6, l6_frame,
                              X_L6W_0.rotation(), theta_bound);

  ik.AddPositionConstraint(
      l7_frame, p_L7oQ_L7, plant->world_frame(),
      p_WQ_start - Eigen::Vector3d::Ones() * position_tolerance,
      p_WQ_start + Eigen::Vector3d::Ones() * position_tolerance);

  ik.get_mutable_prog()->AddLinearConstraint(ik.q()[6] ==
                                             -M_PI / 2 + yaw_angle);

  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_initial_guess);

  solver->Solve(ik.prog(), {}, {}, result.get());
  DRAKE_THROW_UNLESS(result->is_success());

  q0 = result->GetSolution(ik.q());

  // Print q_knots1 to the screen and write it to a file.
  cout << q0 << endl;
  return q0;
}

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
  q0 << CalcInitialJointAngles();
  auto Q_WT = math::RollPitchYawd(M_PI, 0, -M_PI / 2).ToQuaternion();

  Eigen::VectorXd t_knots1(4);
  const double one_way_time = 3;
  t_knots1 << 0, one_way_time, one_way_time + 3, 2 * one_way_time + 3;
  Eigen::MatrixXd xyz_knots(3, 4);
  xyz_knots.col(0) << 0, 0, 0;
  xyz_knots.col(1) << 0, 0, -0.22;
  xyz_knots.col(2) << 0, 0, -0.22;
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