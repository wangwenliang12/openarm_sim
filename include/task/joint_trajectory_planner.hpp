#ifndef OPENARM_JOINT_TRAJECTORY_PLANNER_HPP
#define OPENARM_JOINT_TRAJECTORY_PLANNER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

class JointTrajectoryPlanner {
public:
    enum class PlanMode {
        InterpolateOnly,
        InterpolateThenRefine,
        RefineOnly,
    };

    struct TrajectoryPlan {
        bool active = false;
        PlanMode mode = PlanMode::InterpolateThenRefine;
        bool refine_phase = false;
        double start_time = 0.0;
        double duration = 2.0;
        Eigen::VectorXd start_q;
        Eigen::VectorXd goal_q;
        int owner_state_id = -1;
        Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
        Eigen::Quaterniond target_quat = Eigen::Quaterniond::Identity();
    };

    void plan(const Eigen::VectorXd& start_q,
              const Eigen::VectorXd& goal_q,
              double duration, double now,
              PlanMode mode, int owner_state_id,
              const Eigen::Vector3d& target_pos,
              const Eigen::Quaterniond& target_quat);

    Eigen::VectorXd interpolate(double now) const;
    bool isInterpolationDone(double now) const;

    void clear();
    void enterRefinePhase();
    const TrajectoryPlan& currentPlan() const;

    bool isActiveForState(int state_id) const;

private:
    TrajectoryPlan plan_;
};

#endif
