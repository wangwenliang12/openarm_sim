#ifndef OPENARM_MOTION_REQUEST_HPP
#define OPENARM_MOTION_REQUEST_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

enum class GoalType {
    JointGoal,
    CartesianPoseGoal,
};

struct MotionRequest {
    int owner_state_id = -1;
    GoalType goal_type = GoalType::JointGoal;

    Eigen::VectorXd start_q;
    Eigen::VectorXd joint_goal_q;

    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_quat = Eigen::Quaterniond::Identity();

    double max_planning_time = 0.2;
    double max_joint_velocity = 0.25;
    double max_joint_acceleration = 0.5;
};

#endif
