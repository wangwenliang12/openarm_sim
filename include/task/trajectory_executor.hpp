#ifndef OPENARM_TRAJECTORY_EXECUTOR_HPP
#define OPENARM_TRAJECTORY_EXECUTOR_HPP

#include "task/planned_trajectory.hpp"

#include <Eigen/Core>

class TrajectoryExecutor {
public:
    void clear();
    void setTrajectory(const PlannedTrajectory& trajectory, double start_time);

    bool hasActiveTrajectory() const;
    int ownerStateId() const;

    Eigen::VectorXd sample(double now) const;

private:
    PlannedTrajectory trajectory_;
    double start_time_ = 0.0;
    bool active_ = false;
};

#endif
