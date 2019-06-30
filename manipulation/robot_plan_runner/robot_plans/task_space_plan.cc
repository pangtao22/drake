#include "drake/manipulation/robot_plan_runner/robot_plans/task_space_plan.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using Eigen::VectorXd;
using std::cout;
using std::endl;

const char kIiwaSdf[] =
    "drake/manipulation/models/iiwa_description/iiwa7/"
    "iiwa7_no_collision.sdf";

TaskSpacePlan::TaskSpacePlan()
    : PlanBase(PlanType::kTaskSpacePlan, 7),
      plant_(std::make_unique<multibody::MultibodyPlant<double>>()),
      kp_translation(Eigen::Array3d(100, 100, 100)),
      kp_rotation(Eigen::Array3d(50, 50, 50)),
      task_dimension_(6) {
  // Constructs MultibodyPlant of iiwa7, which is used for Jacobian
  // calculations.
  multibody::Parser parser(plant_.get());
  std::string iiwa_sdf_path = FindResourceOrThrow(kIiwaSdf);
  robot_model_ = parser.AddModelFromFile(iiwa_sdf_path, "iiwa");

  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("iiwa_link_0"));
  plant_->Finalize();
  plant_context_ = plant_->CreateDefaultContext();
  task_frame_idx_ = plant_->GetFrameByName("iiwa_link_7").index();
  Jv_WTq_.resize(task_dimension_, num_positions_);
  x_dot_desired_.resize(task_dimension_);
};

void TaskSpacePlan::UpdatePositionError(
    double t, const PlanData& plan_data,
    const Eigen::Ref<const Eigen::Vector3d>& p_WoQ_W) const {
  if (!p_WoQ_W_t0_) {
    p_WoQ_W_t0_ = std::make_unique<Eigen::Vector3d>(p_WoQ_W);
  }
  const auto p_WoQ_W_ref =
      plan_data.ee_data.value().ee_xyz_traj.value(t) + *p_WoQ_W_t0_;
  err_xyz_ = p_WoQ_W_ref - p_WoQ_W;
};

void TaskSpacePlan::UpdateOrientationError(
    double t, const PlanData& plan_data, const Eigen::Quaterniond& Q_WT) const {
  const auto Q_WT_ref = plan_data.ee_data.value().ee_quat_traj.value(t);
  Q_TTr_ = Q_WT.inverse() * Q_WT_ref;
};

void TaskSpacePlan::UpdateKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v, double t,
    const PlanData& plan_data) const {

};

void TaskSpacePlan::Step(const Eigen::Ref<const Eigen::VectorXd>& q,
                         const Eigen::Ref<const Eigen::VectorXd>& v,
                         const Eigen::Ref<const Eigen::VectorXd>&,
                         double control_period, double t,
                         const PlanData& plan_data, EigenPtr<VectorXd> q_cmd,
                         EigenPtr<VectorXd> tau_cmd) const {
  if (plan_data.plan_type != this->get_plan_type()) {
    throw std::runtime_error("Mismatch between Plan and PlanData.");
  }

  // Update q and v in plant_context_, which is owned by this class.
  plant_->SetPositions(plant_context_.get(), robot_model_, q);
  plant_->SetVelocities(plant_context_.get(), robot_model_, v);

  // Update Kinematics.
  const auto X_WT =
      plant_->CalcRelativeTransform(*plant_context_, plant_->world_frame(),
                                    plant_->get_frame(task_frame_idx_));
  const auto& p_ToQ_T = plan_data.ee_data.value().p_ToQ_T;
  const auto p_WoQ_W = X_WT * p_ToQ_T;
  const auto Q_WT = X_WT.rotation().ToQuaternion();

  plant_->CalcFrameGeometricJacobianExpressedInWorld(
      *plant_context_, plant_->get_frame(task_frame_idx_), p_ToQ_T, &Jv_WTq_);

  // Update errors.
  this->UpdatePositionError(t, plan_data, p_WoQ_W);
  this->UpdateOrientationError(t, plan_data, Q_WT);

  // Update x_dot_desired.
  x_dot_desired_.tail(3) = kp_translation * err_xyz_.array();
    // plan_data.ee_data.value().ee_xyz_dot_traj.value(t).array() +
  x_dot_desired_.head(3) = Q_WT * (kp_rotation * Q_TTr_.vec().array()).matrix();

  auto svd = Jv_WTq_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(0.01);
  const Eigen::VectorXd q_dot_desired = svd.solve(x_dot_desired_);
  *q_cmd = q + q_dot_desired * control_period;
  *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
