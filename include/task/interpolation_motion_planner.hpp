#ifndef OPENARM_INTERPOLATION_MOTION_PLANNER_HPP
#define OPENARM_INTERPOLATION_MOTION_PLANNER_HPP

#include "robot/robot_kinematics.hpp"
#include "task/motion_planner.hpp"

class InterpolationMotionPlanner : public IMotionPlanner {
public:
    explicit InterpolationMotionPlanner(RobotKinematics& kinematics);

    PlannedTrajectory createPlan(const MotionRequest& request) override;

private:
    Eigen::VectorXd resolveGoalQ(const MotionRequest& request) const;

    RobotKinematics& kinematics_;
};

#endif
