#include "task/ompl_motion_planner.hpp"

#include <algorithm>
#include <cmath>

#ifdef OPENARM_HAVE_OMPL
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
#endif

OmplMotionPlanner::OmplMotionPlanner(RobotKinematics& kinematics)
    : kinematics_(kinematics) {}

PlannedTrajectory OmplMotionPlanner::createPlan(const MotionRequest& request) {
#ifdef OPENARM_HAVE_OMPL
    PlannedTrajectory trajectory;
    trajectory.owner_state_id = request.owner_state_id;

    if (request.start_q.size() == 0) {
        return trajectory;
    }

    const Eigen::VectorXd goal_q = resolveGoalQ(request);
    if (goal_q.size() != request.start_q.size()) {
        return trajectory;
    }

    auto state_space = std::make_shared<ob::RealVectorStateSpace>(kinematics_.rightArmDof());
    ob::RealVectorBounds bounds(kinematics_.rightArmDof());
    const std::vector<double> lower = kinematics_.rightArmLowerBounds();
    const std::vector<double> upper = kinematics_.rightArmUpperBounds();
    for (int i = 0; i < kinematics_.rightArmDof(); ++i) {
        bounds.setLow(i, lower[static_cast<size_t>(i)]);
        bounds.setHigh(i, upper[static_cast<size_t>(i)]);
    }
    state_space->setBounds(bounds);

    og::SimpleSetup setup(state_space);
    setup.setStateValidityChecker([](const ob::State* /*state*/) {
        return true;
    });

    const Eigen::VectorXd start_right = kinematics_.extractRightArmQ(request.start_q);
    const Eigen::VectorXd goal_right = kinematics_.extractRightArmQ(goal_q);

    ob::ScopedState<ob::RealVectorStateSpace> start(state_space);
    ob::ScopedState<ob::RealVectorStateSpace> goal(state_space);
    for (int i = 0; i < kinematics_.rightArmDof(); ++i) {
        start[i] = start_right[i];
        goal[i] = goal_right[i];
    }

    setup.setStartAndGoalStates(start, goal);
    setup.setPlanner(std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));

    const ob::PlannerStatus solved = setup.solve(request.max_planning_time);
    if (!solved) {
        return trajectory;
    }

    setup.simplifySolution();

    og::PathGeometric path = setup.getSolutionPath();
    path.interpolate(std::max<std::size_t>(2, path.getStateCount()));

    std::vector<Eigen::VectorXd> full_path;
    full_path.reserve(path.getStateCount());
    for (std::size_t idx = 0; idx < path.getStateCount(); ++idx) {
        const auto* state = path.getState(idx)->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd right_q(kinematics_.rightArmDof());
        for (int j = 0; j < kinematics_.rightArmDof(); ++j) {
            right_q[j] = state->values[j];
        }
        full_path.push_back(kinematics_.mergeRightArmQ(right_q, request.start_q));
    }

    return parameterizePath(full_path, request.owner_state_id, request.max_joint_velocity);
#else
    (void)request;
    return PlannedTrajectory{};
#endif
}

Eigen::VectorXd OmplMotionPlanner::resolveGoalQ(const MotionRequest& request) const {
    if (request.goal_type == GoalType::JointGoal) {
        return request.joint_goal_q;
    }
    return kinematics_.solveRightArmIK(
        request.target_pos, request.target_quat, request.start_q);
}

PlannedTrajectory OmplMotionPlanner::parameterizePath(
    const std::vector<Eigen::VectorXd>& path,
    int owner_state_id,
    double max_joint_velocity) const {
    PlannedTrajectory trajectory;
    trajectory.owner_state_id = owner_state_id;
    if (path.empty()) {
        return trajectory;
    }

    trajectory.valid = true;
    trajectory.points.reserve(path.size());

    double accumulated_time = 0.0;
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) {
            const double max_delta = (path[i] - path[i - 1]).cwiseAbs().maxCoeff();
            accumulated_time += max_delta / std::max(1e-3, max_joint_velocity);
        }

        TrajectoryPoint point;
        point.time_from_start = accumulated_time;
        point.q = path[i];
        trajectory.points.push_back(point);
    }

    trajectory.duration = accumulated_time;
    if (trajectory.points.size() == 1) {
        trajectory.duration = 0.0;
    }
    return trajectory;
}
