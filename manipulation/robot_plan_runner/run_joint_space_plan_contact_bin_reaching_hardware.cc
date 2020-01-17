
#include <fstream>
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

using Eigen::Vector3d;
using Eigen::VectorXd;
using manipulation::robot_plan_runner::robot_plans::PlanData;
using manipulation::robot_plan_runner::robot_plans::PlanType;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::cout;
using std::endl;
using std::vector;
using trajectories::PiecewisePolynomial;

PiecewisePolynomial<double> CalcIiwaTrajectory(
    multibody::MultibodyPlant<double> const* const plant, double duration) {
  const int n_knots = 11;
  const int nq = 7;  // number of joints.
  const double theta_bound = 0.001;
  const double position_tolerance = 0.005;

  const Eigen::Vector3d p_WQ_start(0.30, -0.38 - 0.055, 0);
  const double yaw_angle = -M_PI / 16;
  const auto R_WL6 =
      math::RollPitchYawd(0, -M_PI, yaw_angle).ToRotationMatrix();
  const Eigen::Vector3d p_WQ_offset(0, 0, -0.25);
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

  Eigen::MatrixXd q_knots1(nq, n_knots);
  Eigen::VectorXd t_knots1(n_knots);

  auto solver = std::make_unique<solvers::SnoptSolver>();
  auto result = std::make_unique<solvers::MathematicalProgramResult>();

  for (int i = 0; i < n_knots; i++) {
    multibody::InverseKinematics ik(*plant);

    ik.AddOrientationConstraint(plant->world_frame(), R_WL6, l6_frame,
                                X_L6W_0.rotation(), theta_bound);

    const Eigen::Vector3d p_WQ = p_WQ_start + p_WQ_offset / (n_knots - 1) * i;
    ik.AddPositionConstraint(
        l7_frame, p_L7oQ_L7, plant->world_frame(),
        p_WQ - Eigen::Vector3d::Ones() * position_tolerance,
        p_WQ + Eigen::Vector3d::Ones() * position_tolerance);

    ik.get_mutable_prog()->AddLinearConstraint(ik.q()[6] ==
                                               -M_PI / 2 + yaw_angle);

    if (i == 0) {
      ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_initial_guess);
    } else {
      ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_knots1.col(i - 1));
    }
    solver->Solve(ik.prog(), {}, {}, result.get());
    cout << i << ": " << result->get_solution_result() << endl << p_WQ << endl;
    DRAKE_THROW_UNLESS(result->is_success());

    q_knots1.col(i) = result->GetSolution(ik.q());

    t_knots1(i) = duration / (n_knots - 1) * i;
  }

  // Print q_knots1 to the screen and write it to a file.
  cout << q_knots1 << endl;
  std::ofstream file("q_knots.csv");
  const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                  ", ", "\n");
  file << q_knots1.format(CSVFormat);
  file.close();

  return trajectories::PiecewisePolynomial<double>::Cubic(
      t_knots1, q_knots1, Eigen::VectorXd::Zero(nq), Eigen::VectorXd::Zero(nq));
}

int run_plan() {
  // Get a joint space trajecotry by solving IK.
  auto plant = std::make_unique<multibody::MultibodyPlant<double>>();
  manipulation::robot_plan_runner::robot_plans::SetupIiwaControllerPlant(
      plant.get());
  const int nq = plant->num_positions();

  // plan1 runs joint space contact plan.
  PlanData plan1;
  const double plan1_duration = 16;
  plan1.plan_type = PlanType::kJointSpacePlanContact;
  plan1.joint_traj = CalcIiwaTrajectory(plant.get(), plan1_duration);

  // plan0 holds at the starting pose of the contact-aware plan for 1 second.
  PlanData plan0;

  Eigen::VectorXd t_knots0(2);
  t_knots0 << 0, 1;

  Eigen::MatrixXd q_knots(nq, 2);
  q_knots.col(0) = plan1.joint_traj.value().value(0);
  q_knots.col(1) = q_knots.col(0);

  auto qtraj = trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      t_knots0, q_knots);

  plan0.plan_type = PlanType::kJointSpacePlan;
  plan0.joint_traj = qtraj;

  vector<PlanData> plan_list{plan0, plan1};

  // Construct plan runner hardware interface.
  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list,
                                                                   true);
  plan_runner.SaveGraphvizStringToFile();

  // Run simulation.
  plan_runner.Run();

  return 0;
};

}  // namespace
}  // namespace drake

int main() { return drake::run_plan(); }
