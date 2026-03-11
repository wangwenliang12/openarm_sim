#ifndef OPENARM_OMPL_MOTION_PLANNER_HPP
#define OPENARM_OMPL_MOTION_PLANNER_HPP

#include "robot/robot_kinematics.hpp"
#include "task/motion_planner.hpp"

#include <vector>

class OmplMotionPlanner : public IMotionPlanner {
public:
    explicit OmplMotionPlanner(RobotKinematics& kinematics);

    PlannedTrajectory createPlan(const MotionRequest& request) override;

private:
    Eigen::VectorXd resolveGoalQ(const MotionRequest& request) const;
    PlannedTrajectory parameterizePath(const std::vector<Eigen::VectorXd>& path,
                                       int owner_state_id,
                                       double max_joint_velocity) const;

    RobotKinematics& kinematics_;
};

#endif
