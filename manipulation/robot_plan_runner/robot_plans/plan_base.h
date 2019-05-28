#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

enum class PlanType { kJointSpacePlan, kTaskSpacePlan, kEmptyPlan };

struct PlanData {
  PlanType plan_type{PlanType::kEmptyPlan};
  // plan signature can be 1,2,3 (as in the case of running simulations)
  // or timestamp (as in the case of receiving LCM messages).
  long plan_signature{-1};
  optional<trajectories::PiecewisePolynomial<double>> joint_traj;
  struct EeData {
    // Coordinates of point Q expressed in frame T (task frame).
    Eigen::Vector3d p_ToQ_T;
    // Reference Cartesian coordinates of point Q in world frame relative to
    // the position of Q at the beginning of the plan:
    // p_WoQ_W_ref = ee_xyz_traj.value(t) + p_WoQ_W_t0
    trajectories::PiecewisePolynomial<double> ee_xyz_traj;
    // Reference orientation (\in SO(3)) of frame T in world frame as a
    // quaternion: Q_WT_ref.
    trajectories::PiecewiseQuaternionSlerp<double> ee_quat_traj;
  };
  optional<EeData> ee_data;
};

class PlanBase {
 public:
  PlanBase(PlanType plan_type, int num_positions)
      : num_positions_(num_positions), plan_type_(plan_type) {};
  virtual ~PlanBase() = default;

  virtual void Step(const Eigen::Ref<const Eigen::VectorXd> &q,
                    const Eigen::Ref<const Eigen::VectorXd> &v,
                    const Eigen::Ref<const Eigen::VectorXd> &tau_external,
                    double control_period,
                    double t,
                    const PlanData &plan_data,
                    EigenPtr<Eigen::VectorXd> q_commanded,
                    EigenPtr<Eigen::VectorXd> tau_commanded) const = 0;
  PlanType get_plan_type() const { return plan_type_; };

 protected:
  const int num_positions_;
  const PlanType plan_type_;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
