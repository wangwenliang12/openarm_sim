#include "task/joint_trajectory_planner.hpp"

#include <algorithm>
#include <cmath>

void JointTrajectoryPlanner::plan(const Eigen::VectorXd& start_q,
                                   const Eigen::VectorXd& goal_q,
                                   double duration, double now,
                                   PlanMode mode, int owner_state_id,
                                   const Eigen::Vector3d& target_pos,
                                   const Eigen::Quaterniond& target_quat) {
    plan_.active = true;
    plan_.mode = mode;
    plan_.refine_phase = (mode == PlanMode::RefineOnly);
    plan_.start_time = now;
    plan_.duration = duration;
    plan_.start_q = start_q;
    plan_.goal_q = goal_q;
    plan_.owner_state_id = owner_state_id;
    plan_.target_pos = target_pos;
    plan_.target_quat = target_quat;
}

Eigen::VectorXd JointTrajectoryPlanner::interpolate(double now) const {
    if (!plan_.active || plan_.refine_phase) {
        return plan_.goal_q;
    }
    const double alpha = std::clamp(
        (now - plan_.start_time) / std::max(1e-3, plan_.duration), 0.0, 1.0);
    const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
    return plan_.start_q + smooth_alpha * (plan_.goal_q - plan_.start_q);
}

bool JointTrajectoryPlanner::isInterpolationDone(double now) const {
    if (!plan_.active) return true;
    if (plan_.refine_phase) return true;
    return (now - plan_.start_time) >= plan_.duration;
}

void JointTrajectoryPlanner::clear() {
    plan_.active = false;
    plan_.refine_phase = false;
    plan_.owner_state_id = -1;
}

void JointTrajectoryPlanner::enterRefinePhase() {
    plan_.refine_phase = true;
}

const JointTrajectoryPlanner::TrajectoryPlan& JointTrajectoryPlanner::currentPlan() const {
    return plan_;
}

bool JointTrajectoryPlanner::isActiveForState(int state_id) const {
    return plan_.active && plan_.owner_state_id == state_id;
}
