#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
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

VectorXd CalcStartingJointAngles(
    const RotationMatrixd& R_WL7, const Eigen::Ref<const Vector3d>& p_WQ_start,
    const Eigen::Ref<const Vector3d>& p_BQ,
    const Eigen::Ref<const VectorXd>& q_initial_guess) {
  // Constructs MultibodyPlant.
  auto plant = std::make_unique<multibody::MultibodyPlant<double>>();
  manipulation::robot_plan_runner::robot_plans::SetupIiwaControllerPlant(
      plant.get());

  const double theta_bound = 0.001;
  const double position_tolerance = 0.005;

  auto solver = std::make_unique<solvers::SnoptSolver>();
  auto result = std::make_unique<solvers::MathematicalProgramResult>();

  multibody::InverseKinematics ik(*plant);
  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_initial_guess);
  const auto& l7_frame = plant->GetFrameByName("iiwa_link_7");

  ik.AddPositionConstraint(
      l7_frame, p_BQ, plant->world_frame(),
      p_WQ_start - Eigen::Vector3d::Ones() * position_tolerance,
      p_WQ_start + Eigen::Vector3d::Ones() * position_tolerance);
  ik.AddOrientationConstraint(plant->world_frame(), R_WL7, l7_frame,
                              math::RotationMatrixd::Identity(), theta_bound);
  solver->Solve(ik.prog(), {}, {}, result.get());
  cout << result->get_solution_result() << endl;
  DRAKE_THROW_UNLESS(result->is_success());

  return result->GetSolution(ik.q());
};

int run_plan() {
  // Create plans.
  // Plan1 is the hybrid force-position plan.
  PlanData plan1;
  plan1.plan_type = PlanType ::kHybridForcePositionPlan;

  PlanData::EeData ee_data;
  ee_data.p_ToQ_T << 0, 0.05 / std::sqrt(2), 0.075 + 0.05 / std::sqrt(2);

  Eigen::MatrixXd xyz_knots(2, 3);
  xyz_knots.col(0) << 0, 0;
  xyz_knots.col(2) << 0, -0.20;
  xyz_knots.col(1) = (xyz_knots.col(0) + xyz_knots.col(2)) / 2;
  Eigen::Vector3d t_knots1(0, 8, 16);
  ee_data.ee_xyz_traj = trajectories::PiecewisePolynomial<double>::Cubic(
      t_knots1, xyz_knots, Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2));
  ee_data.ee_xyz_dot_traj = ee_data.ee_xyz_traj.derivative(1);

  auto Q_WT = RollPitchYawd(0, M_PI * 1.25, 0).ToQuaternion();
  vector<double> t_knots_v{t_knots1[0], t_knots1[1], t_knots1[2]};
  vector<Eigen::Quaterniond> quaternions{Q_WT, Q_WT, Q_WT};
  ee_data.ee_quat_traj =
      trajectories::PiecewiseQuaternionSlerp<double>(t_knots_v, quaternions);

  plan1.ee_data = ee_data;

  PlanData::HybridTaskDefinition task_definition;
  // x,y,z axes of C are aligned with those of the world frame.
  task_definition.R_WC = Eigen::Matrix3d::Identity();

  task_definition.force_controlled_axes.push_back(2);   // world z
  task_definition.motion_controlled_axes.push_back(0);  // world x
  task_definition.motion_controlled_axes.push_back(1);  // world y

  plan1.hybrid_task_definition = task_definition;

  // Plan0 goes to the starting pose of the hybrid force position plan.
  PlanData plan0;
  const int nq = 7;
  VectorXd t_knots0(2);
  t_knots0 << 0, 1;

  VectorXd q_initial_guess(nq);
  q_initial_guess << -0.4532, 1.1335, -0.0479, -1.3982, -0.3034, 1.3395,
      -0.3055;

  Eigen::MatrixXd q_knots(nq, 2);
  q_knots.col(0) = CalcStartingJointAngles(
      RotationMatrixd(Q_WT),
      Eigen::Vector3d(0.40, 0, 0.03),
      ee_data.p_ToQ_T,
      q_initial_guess);

  q_knots.col(1) = q_knots.col(0);

  plan0.plan_type = PlanType::kJointSpacePlan;
  plan0.joint_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(t_knots0, q_knots);

  // Construct plan runner hardware interface.
  vector<PlanData> plan_list{plan0, plan1};

  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list,
                                                                   false);
  //  plan_runner.SaveGraphvizStringToFile();

  // Run simulation.
  plan_runner.Run();

  return 0;
}

}  // namespace
}  // namespace drake

int main() { return drake::run_plan(); }