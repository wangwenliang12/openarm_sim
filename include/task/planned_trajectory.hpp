#ifndef OPENARM_PLANNED_TRAJECTORY_HPP
#define OPENARM_PLANNED_TRAJECTORY_HPP

#include <Eigen/Core>

#include <vector>

struct TrajectoryPoint {
    double time_from_start = 0.0;
    Eigen::VectorXd q;
};

struct PlannedTrajectory {
    bool valid = false;
    int owner_state_id = -1;
    double duration = 0.0;
    std::vector<TrajectoryPoint> points;
};

#endif
