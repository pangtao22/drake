#pragma once

#include <Eigen/Core>

#include "drake/manipulation/robot_plan_runner/robot_plans/contact_force_estimator.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

class HybridForcePositionPlan : public PlanBase {
 public:
  HybridForcePositionPlan();

  void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
            const Eigen::Ref<const Eigen::VectorXd>& v,
            const Eigen::Ref<const Eigen::VectorXd>& tau_external,
            double control_period, double t, const PlanData& plan_data,
            const robot_plans::ContactInfo& contact_info,
            EigenPtr<Eigen::VectorXd> q_cmd,
            EigenPtr<Eigen::VectorXd> tau_cmd) const override;

 private:
  Eigen::MatrixXd CalcSelectorMatrix(
      const std::vector<unsigned int>& axes) const;

  const Eigen::Array3d kp_translation_;
  const Eigen::Array3d kp_rotation_;
  const double velocity_cost_weight_;

  std::unique_ptr<multibody::MultibodyPlant<double>> plant_;
  multibody::FrameIndex task_frame_idx_;
  multibody::ModelInstanceIndex robot_model_;
  mutable std::unique_ptr<systems::Context<double>> plant_context_;
  mutable Eigen::MatrixXd Jv_WTq_;

  Eigen::ArrayXd joint_stiffness_;
  Eigen::MatrixXd dq_weight_;

  std::unique_ptr<solvers::MathematicalProgramResult> prog_result_;
  solvers::GurobiSolver solver_;

  std::unique_ptr<ContactForceEstimator> contact_force_estimator_;
  std::unique_ptr<FirstOrderSystem<double>> f_contact_ref_;

  std::unique_ptr<FirstOrderSystem<double>> v_translation_norm_limit_;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake