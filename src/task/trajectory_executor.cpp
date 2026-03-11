#include "task/trajectory_executor.hpp"

#include <algorithm>

void TrajectoryExecutor::clear() {
    trajectory_ = PlannedTrajectory{};
    start_time_ = 0.0;
    active_ = false;
}

void TrajectoryExecutor::setTrajectory(const PlannedTrajectory& trajectory, double start_time) {
    trajectory_ = trajectory;
    start_time_ = start_time;
    active_ = trajectory.valid && !trajectory.points.empty();
}

bool TrajectoryExecutor::hasActiveTrajectory() const {
    return active_;
}

int TrajectoryExecutor::ownerStateId() const {
    return trajectory_.owner_state_id;
}

Eigen::VectorXd TrajectoryExecutor::sample(double now) const {
    if (!active_ || trajectory_.points.empty()) {
        return Eigen::VectorXd{};
    }

    const double elapsed = std::max(0.0, now - start_time_);
    if (elapsed <= trajectory_.points.front().time_from_start) {
        return trajectory_.points.front().q;
    }
    if (elapsed >= trajectory_.points.back().time_from_start) {
        return trajectory_.points.back().q;
    }

    const auto upper = std::upper_bound(
        trajectory_.points.begin(), trajectory_.points.end(), elapsed,
        [](double t, const TrajectoryPoint& point) {
            return t < point.time_from_start;
        });
    const auto lower = upper - 1;

    const double t0 = lower->time_from_start;
    const double t1 = upper->time_from_start;
    const double alpha = (elapsed - t0) / std::max(1e-6, t1 - t0);
    return lower->q + alpha * (upper->q - lower->q);
}
