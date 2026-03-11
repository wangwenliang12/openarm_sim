#ifndef OPENARM_MANIPULATE_TASK_HPP
#define OPENARM_MANIPULATE_TASK_HPP

#include "robot/robot_kinematics.hpp"
#include "state_machine/grasp_state_machine.hpp"
#include "task/motion_planner.hpp"
#include "task/trajectory_executor.hpp"

#include <memory>
#include <string>
#include <unordered_map>

struct ControlLimits {
    double arm_joint_velocity = 0.25;
    double finger_velocity = 0.02;
};

class ManipulateTask {
public:
    using NamedPoseMap = std::unordered_map<std::string, std::unordered_map<std::string, double>>;

    ManipulateTask(const std::string& urdf_path,
                   const std::string& ee_frame_name,
                   const std::vector<std::string>& right_arm_joints,
                   const std::vector<std::string>& left_arm_joints);

    void setRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

    pinocchio::SE3 getEndEffectorPose();

    void loadNamedPoses(const NamedPoseMap& poses);
    bool isNamedPoseReached(const std::string& name, double tol) const;

    void onStateTransition(int prev_state_id, int next_state_id, double now);
    bool ensureTrajectoryForState(const GraspStateMachine::Command& command,
                                  int state_id,
                                  double now);
    Eigen::VectorXd sampleJointTargets(double now) const;

    Eigen::VectorXd smoothJointTargets(
        const Eigen::VectorXd& q_target,
        const ControlLimits& limits, double dt) const;

    void setJointAngles(const Eigen::VectorXd& q_target);

    RobotKinematics& kinematics();
    const RobotKinematics& kinematics() const;

private:
    MotionRequest buildMotionRequest(const GraspStateMachine::Command& command,int state_id) const;
    Eigen::VectorXd getNamedPoseTargets(const std::string& name) const;
    bool stateRequiresArmPlan(const GraspStateMachine::Command& command) const;
    IMotionPlanner& plannerForRequest(const MotionRequest& request) const;
    void captureHoldPosition();
    // 负责IK/FK
    RobotKinematics kinematics_;
    std::unique_ptr<IMotionPlanner> interpolation_planner_;
    std::unique_ptr<IMotionPlanner> ompl_planner_;
    TrajectoryExecutor executor_;
    std::unordered_map<std::string, Eigen::VectorXd> named_pose_targets_;
    Eigen::VectorXd hold_q_;
    bool hold_position_active_ = false;
    mutable Eigen::VectorXd smoothed_q_cmd_;
};

#endif
