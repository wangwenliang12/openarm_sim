#include "task/interpolation_motion_planner.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr int kNumSegments = 20;
}

InterpolationMotionPlanner::InterpolationMotionPlanner(RobotKinematics& kinematics)
    : kinematics_(kinematics) {}

PlannedTrajectory InterpolationMotionPlanner::createPlan(const MotionRequest& request) {
    PlannedTrajectory trajectory;
    trajectory.owner_state_id = request.owner_state_id;

    if (request.start_q.size() == 0) {
        return trajectory;
    }

    const Eigen::VectorXd goal_q = resolveGoalQ(request);
    if (goal_q.size() != request.start_q.size()) {
        return trajectory;
    }

    const double max_delta = (goal_q - request.start_q).cwiseAbs().maxCoeff();
    const double duration = std::max(0.25, max_delta / std::max(1e-3, request.max_joint_velocity));

    trajectory.valid = true;
    trajectory.duration = duration;
    trajectory.points.reserve(static_cast<size_t>(kNumSegments + 1));

    for (int i = 0; i <= kNumSegments; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(kNumSegments);
        const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);

        TrajectoryPoint point;
        point.time_from_start = alpha * duration;
        point.q = request.start_q + smooth_alpha * (goal_q - request.start_q);
        trajectory.points.push_back(point);
    }

    return trajectory;
}

Eigen::VectorXd InterpolationMotionPlanner::resolveGoalQ(const MotionRequest& request) const {
    if (request.goal_type == GoalType::JointGoal) {
        return request.joint_goal_q;
    }
    return kinematics_.solveRightArmIK(
        request.target_pos, request.target_quat, request.start_q);
}
