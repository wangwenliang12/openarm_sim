#ifndef OPENARM_MANIPULATE_TASK_HPP
#define OPENARM_MANIPULATE_TASK_HPP

#include "robot/robot_kinematics.hpp"
#include "state_machine/grasp_state_machine.hpp"
#include "task/joint_trajectory_planner.hpp"

#include <string>
#include <unordered_map>

struct ControlLimits {
    double arm_joint_velocity = 0.25;
    double finger_velocity = 0.02;
};

class ManipulateTask {
public:
    using NamedPoseMap = std::unordered_map<
        std::string, std::unordered_map<std::string, double>>;

    ManipulateTask(const std::string& urdf_path,
                   const std::string& ee_frame_name,
                   const std::vector<std::string>& right_arm_joints,
                   const std::vector<std::string>& left_arm_joints);

    void setRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

    pinocchio::SE3 getEndEffectorPose();

    void loadNamedPoses(const NamedPoseMap& poses);
    Eigen::VectorXd getNamedPoseTargets(const std::string& name) const;
    bool isNamedPoseReached(const std::string& name, double tol) const;

    Eigen::VectorXd computeJointTargets(
        const GraspStateMachine::Command& command,
        int current_state_id,
        double now);

    void onStateTransition(int prev_state_id, int next_state_id, double now);

    Eigen::VectorXd smoothJointTargets(
        const Eigen::VectorXd& q_target,
        const ControlLimits& limits, double dt) const;

    void setJointAngles(const Eigen::VectorXd& q_target);

    RobotKinematics& kinematics();
    const RobotKinematics& kinematics() const;

private:
    JointTrajectoryPlanner::PlanMode planModeForState(int state_id) const;
    double planDurationForState(int state_id) const;

    void ensurePlanForState(const GraspStateMachine::Command& command,
                            int state_id, double now);

    RobotKinematics kinematics_;
    JointTrajectoryPlanner planner_;
    std::unordered_map<std::string, Eigen::VectorXd> named_pose_targets_;
    mutable Eigen::VectorXd smoothed_q_cmd_;
};

#endif
