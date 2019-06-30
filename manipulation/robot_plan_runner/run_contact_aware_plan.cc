
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/manipulation/robot_plan_runner/plan_sender.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace {

using examples::manipulation_station::ManipulationStation;
using robot_plans::PlanData;
using robot_plans::PlanType;
using std::cout;
using std::endl;
using std::vector;

/*
 * This example connects a ManipulationStation to a RobotPlanRunner and a
 * PlanSender, and then runs a JointSpacePlan.
 */
int test_joint_space_plan() {
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
  plan1.plan_type = PlanType::kTaskSpacePlan;

  PlanData::EeData ee_data;
  ee_data.p_ToQ_T.setZero();

  Eigen::Vector3d t_knots2(0, 2.5, 5);

  Eigen::MatrixXd xyz_knots(3, 3);
  xyz_knots.col(0) << 0, 0, 0;
  xyz_knots.col(1) << 0, 0.125, 0.075;
  xyz_knots.col(2) << 0, 0.25, 0.15;

  ee_data.ee_xyz_traj =
      trajectories::PiecewisePolynomial<double>::Cubic(
          t_knots2, xyz_knots, Eigen::VectorXd::Zero(3),
          Eigen::VectorXd::Zero(3));
  ee_data.ee_xyz_dot_traj = ee_data.ee_xyz_traj.derivative(1);

  auto Q_WT = math::RollPitchYawd(0, M_PI * 1.25, 0).ToQuaternion();

  vector<double> t_knots_v{0, 2.5, 5};
  vector<Eigen::Quaterniond> quaternions{Q_WT, Q_WT, Q_WT};

  ee_data.ee_quat_traj =
      trajectories::PiecewiseQuaternionSlerp<double>(t_knots_v, quaternions);

  plan1.ee_data = ee_data;
  vector<PlanData> plan_list{plan0, plan1};


  systems::DiagramBuilder<double> builder;

  // Set up ManipulationStation.
  auto station = builder.template AddSystem<ManipulationStation<double>>();
  station->SetupDefaultStation();
  station->Finalize();

  // Set up PlanRunner.
  auto plan_runner = builder.template AddSystem<RobotPlanRunner>(true, 0.005);

  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  plan_runner->GetInputPort("iiwa_position_measured"));
  builder.Connect(station->GetOutputPort("iiwa_velocity_estimated"),
                  plan_runner->GetInputPort("iiwa_velocity_estimated"));
  builder.Connect(station->GetOutputPort("iiwa_torque_external"),
                  plan_runner->GetInputPort("iiwa_torque_external"));
  builder.Connect(plan_runner->GetOutputPort("iiwa_position_command"),
                  station->GetInputPort("iiwa_position"));
  builder.Connect(plan_runner->GetOutputPort("iiwa_torque_command"),
                  station->GetInputPort("iiwa_feedforward_torque"));

  // Set up PlanSender.
  auto plan_sender = builder.template AddSystem<PlanSender>(plan_list);
  builder.Connect(plan_sender->GetOutputPort("plan_data"),
                  plan_runner->GetInputPort("plan_data"));
  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  plan_sender->GetInputPort("q"));

  // Add a visualizer.
  geometry::ConnectDrakeVisualizer(&builder, station->get_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();
  auto& station_context =
      diagram->GetMutableSubsystemContext(*station, &context);

  station->GetInputPort("wsg_force_limit").FixValue(&station_context, 40.0);
  station->GetInputPort("wsg_position").FixValue(&station_context, 0.05);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(plan_sender->get_all_plans_duration());

  return 0;
};

}  // namespace
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake

int main() {
  return drake::manipulation::robot_plan_runner::test_joint_space_plan();
};