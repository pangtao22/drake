
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
using math::RotationMatrixd;
using std::cout;
using std::endl;
using std::vector;
using trajectories::PiecewisePolynomial;

Eigen::VectorXd CalcInitialJointAngles() {
  auto plant = std::make_unique<multibody::MultibodyPlant<double>>(1e-3);
  manipulation::robot_plan_runner::robot_plans::SetupIiwaControllerPlant(
      plant.get());
  const Eigen::Vector3d p_L7oQ_L7(0, 0, 0.075);

  const int nq = 7;  // number of joints.
  const double theta_bound = 0.001;
  const double position_tolerance = 0.0001;

  const Eigen::Vector3d p_WQ_start(0.247, -0.542, 0.195);
  const auto R_WL7 =
      math::RollPitchYawd(2.58, 0, -M_PI / 2).ToRotationMatrix();

  Eigen::VectorXd q_initial_guess(nq);
  q_initial_guess << -1.43, 1.13, 0.79, -1.30, -1.17, 1.60, -2.05;

  const auto& l7_frame = plant->GetFrameByName("iiwa_link_7");

  auto context = plant->CreateDefaultContext();
  Eigen::VectorXd q0(nq);

  auto solver = std::make_unique<solvers::SnoptSolver>();
  auto result = std::make_unique<solvers::MathematicalProgramResult>();

  multibody::InverseKinematics ik(*plant);

  ik.AddOrientationConstraint(plant->world_frame(), R_WL7, l7_frame,
                              RotationMatrixd(), theta_bound);

  ik.AddPositionConstraint(
      l7_frame, p_L7oQ_L7, plant->world_frame(),
      p_WQ_start - Eigen::Vector3d::Ones() * position_tolerance,
      p_WQ_start + Eigen::Vector3d::Ones() * position_tolerance);


  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_initial_guess);

  solver->Solve(ik.prog(), {}, {}, result.get());
  DRAKE_THROW_UNLESS(result->is_success());

  q0 = result->GetSolution(ik.q());

  // Print q_knots1 to the screen and write it to a file.
  cout << q0 << endl;
  return q0;
}

int run_plan() {
  Eigen::VectorXd q0 = CalcInitialJointAngles();

  // plan0 goes to the starting pose of the contact-aware plan.
  PlanData plan0;
  Eigen::VectorXd t_knots0(2);
  t_knots0 << 0, 1;
  Eigen::MatrixXd q_knots(7, 2);
  q_knots.col(0) = q0;
  const auto qtraj0 = trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      t_knots0, q_knots);

  plan0.plan_type = PlanType::kJointSpacePlan;
  plan0.joint_traj = qtraj0;

  // plan1 runs contact-aware plan.
  vector<Eigen::Quaterniond> quaternions{
    math::RollPitchYawd(2.58, 0, -M_PI / 2).ToQuaternion(),
    math::RollPitchYawd(2.58, 0, -M_PI / 2).ToQuaternion(),
    math::RollPitchYawd(2.18, 0, -M_PI / 2).ToQuaternion(),
    math::RollPitchYawd(2.18, 0, -M_PI / 2).ToQuaternion(),
    math::RollPitchYawd(2.18, 0, -M_PI / 2).ToQuaternion()};

  vector<double> t_knots1{0, 3, 8, 9, 15};

  Eigen::MatrixXd xyz_knots(3, 5);
  xyz_knots.col(0) << 0, 0, 0;
  xyz_knots.col(1) << 0, 0, -0.25;
  xyz_knots.col(2) << 0, 0, -0.25;
  xyz_knots.col(3) << 0, 0, -0.25;
  xyz_knots.col(4) << 0, 0, 0;

  Eigen::MatrixXd xyz_dot_knots(3, 5);
  xyz_dot_knots.setZero();

  PlanData plan1;
  plan1.plan_type = PlanType::kTaskSpacePlanContact;

  PlanData::EeData ee_data;
  ee_data.p_ToQ_T << 0, 0, 0.075;
  ee_data.ee_xyz_traj =
      trajectories::PiecewisePolynomial<double>::Cubic(
          Eigen::Map<const Eigen::VectorXd>(t_knots1.data(), t_knots1.size()),
          xyz_knots, xyz_dot_knots);
  ee_data.ee_xyz_dot_traj = ee_data.ee_xyz_traj.derivative(1);
  ee_data.ee_quat_traj =
      trajectories::PiecewiseQuaternionSlerp<double>(t_knots1, quaternions);
  plan1.ee_data = ee_data;

  // Construct plan runner hardware interface.
  vector<PlanData> plan_list{plan0, plan1};
  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(
          plan_list, true, true);
  plan_runner.SaveGraphvizStringToFile();

  // Run simulation.
  plan_runner.Run();

  return 0;
}

}  // namespace
}  // namespace drake

int main() { return drake::run_plan(); }