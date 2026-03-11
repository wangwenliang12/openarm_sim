#ifndef OPENARM_MOTION_PLANNER_HPP
#define OPENARM_MOTION_PLANNER_HPP

#include "task/motion_request.hpp"
#include "task/planned_trajectory.hpp"

class IMotionPlanner {
public:
    virtual ~IMotionPlanner() = default;

    virtual PlannedTrajectory createPlan(const MotionRequest& request) = 0;
};

#endif
