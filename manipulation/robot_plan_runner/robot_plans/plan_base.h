#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_optional.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/contact_force_estimator.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

/*
 * KEmptyPlan has to be the first in the enumeration. This is because in
 * robot_plan_runner, integer values of the enums are used to index input
 * ports of the PortSwitch system. As port 0 is reserved for the port_selector
 * input port, indices of user-defined ports should start with 1.
 *
 * kLastElement is used to detect the end of this enum when it is looped
 * through.
 */
enum class PlanType {
  kEmptyPlan,
  kTaskSpacePlan,
  kTaskSpacePlanContact,
  kJointSpacePlan,
  kJointSpacePlanContact,
  kHybridForcePositionPlan,
  kLastElement
};

struct PlanData {
  PlanType plan_type{PlanType::kEmptyPlan};
  // Plan signature should be different for every unique plan. It can be
  //  1,2,3, ..., as in the case of running simulations, or
  //  the integer timestamp of the robot_plan_t message received.
  long plan_signature{-1};

  // For plans using joint space trajectories.
  optional<trajectories::PiecewisePolynomial<double>> joint_traj;

  // For plans using task space trajectories.
  struct EeData {
    // Coordinates of point Q expressed in frame T (task frame).
    Eigen::Vector3d p_ToQ_T;

    // Reference Cartesian coordinates of point Q in world frame relative to
    // the position of Q at the beginning of the plan:
    // p_WoQ_W_ref = ee_xyz_traj.value(t) + p_WoQ_W_t0
    trajectories::PiecewisePolynomial<double> ee_xyz_traj;

    // Derivative of ee_xyz_traj.
    trajectories::PiecewisePolynomial<double> ee_xyz_dot_traj;

    // Reference orientation (\in SO(3)) of frame T in world frame as a
    // quaternion: Q_WT_ref.
    trajectories::PiecewiseQuaternionSlerp<double> ee_quat_traj;
  };
  optional<EeData> ee_data;

  // For hybrid force-position plans.
  struct HybridTaskDefinition {
    // Coordinates of the three axes of the task frame expressed in world frame.
    // Each column of C is an axis.
    Eigen::Matrix3d R_WC;
    // Indices of the columns of C which are force/motion controlled.
    std::vector<unsigned int> force_controlled_axes{};
    std::vector<unsigned int> motion_controlled_axes{};
  };
  optional<HybridTaskDefinition> hybrid_task_definition;

  // Returns the duration of this plan.
  double get_duration() const;
};

/*
 * Abstract base class for all concrete plans.
 */
class PlanBase {
 public:
  PlanBase(PlanType plan_type, int num_positions)
      : num_positions_(num_positions), plan_type_(plan_type){};
  virtual ~PlanBase() = default;

  /*
   * This function is called in controller subsystems of RobotPlanRunner. It
   * takes current robot state (q, v, tau_external), current time (t) and
   * current plan to be executed (plan_data). It writes q and tau_external
   * commands to the pointers provided.
   */
  virtual void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
                    const Eigen::Ref<const Eigen::VectorXd>& v,
                    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
                    double control_period, double t, const PlanData& plan_data,
                    const robot_plans::ContactInfo& contact_info,
                    EigenPtr<Eigen::VectorXd> q_commanded,
                    EigenPtr<Eigen::VectorXd> tau_commanded) const = 0;
  PlanType get_plan_type() const { return plan_type_; };

 protected:
  void set_plan_type(PlanType plan_type) { plan_type_ = plan_type; };
  void check_plan_type(const PlanData& plan_data) const;
  const int num_positions_;
  PlanType plan_type_;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
