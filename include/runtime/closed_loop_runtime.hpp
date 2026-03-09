#ifndef OPENARM_CLOSED_LOOP_RUNTIME_HPP
#define OPENARM_CLOSED_LOOP_RUNTIME_HPP

#include "publisher/lcm_pose_publisher.hpp"
#include "state_machine/grasp_state_machine.hpp"
#include "subscriber/lcm_pose_subscriber.hpp"

#include <mujoco/mujoco.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <string>
#include <unordered_map>
#include <vector>

class ClosedLoopRuntime {
public:
    explicit ClosedLoopRuntime(const mjModel* model);

    void initialize(const mjModel* model, const mjData* data);
    void step(const mjModel* model, mjData* data);

private:
    struct RobotJointMap {
        std::string name;
        int mj_joint_id = -1;
        int mj_qpos_adr = -1;
        int mj_dof_adr = -1;
        int pin_q_adr = -1;
        int pin_v_adr = -1;
        int actuator_id = -1;
    };

    void setupJointMaps(const mjModel* model);
    void setupActuatorIds(const mjModel* model);
    void updateRobotState(const mjData* data);
    void cacheNamedJointPoses();
    Eigen::VectorXd getNamedJointPoseTargets(const std::string& pose_name) const;
    bool isNamedJointPoseReached(const std::string& pose_name, double tolerance) const;
    Eigen::VectorXd smoothJointTargets(const Eigen::VectorXd& q_target) const;
    double jointVelocityLimitForState() const;
    double fingerVelocityLimitForState() const;
    void updatePregraspPlanIfNeeded(const GraspStateMachine::Command& command,
                                    const pinocchio::SE3& current_pose,
                                    double now);
    Eigen::VectorXd interpolatePregraspJointTargets(double now) const;
    void updateDescendPlanIfNeeded(const GraspStateMachine::Command& command, double now);
    Eigen::VectorXd interpolateDescendJointTargets(double now) const;
    Eigen::VectorXd solveRightArmIkGoal(const Eigen::Vector3d& target_pos,
                                        const Eigen::Quaterniond& target_quat,
                                        const Eigen::VectorXd& seed_q) const;
    void applyRobotGravityCompensation(mjData* data) const;
    pinocchio::SE3 getCurrentPose();
    Eigen::VectorXd computeTaskJointTargets(const Eigen::Vector3d& target_pos,
                                            const Eigen::Quaterniond& target_quat) const;
    void writeArmPositionControls(mjData* data, const Eigen::VectorXd& q_cmd) const;
    void writeFingerControls(mjData* data, double left_cmd, double right_cmd) const;
    void writeHeadControls(mjData* data) const;

    pinocchio::Model pin_model_;
    pinocchio::Data pin_data_;
    pinocchio::FrameIndex active_ee_frame_id_ = 0;
    double control_timestep_ = 0.002;

    LcmPosePublisher publisher_;
    LcmPoseSubscriber subscriber_;
    GraspStateMachine state_machine_;

    std::vector<RobotJointMap> robot_joints_;
    std::vector<int> left_arm_joint_indices_;
    std::vector<int> right_arm_joint_indices_;
    Eigen::VectorXd q_robot_;
    Eigen::VectorXd dq_robot_;
    Eigen::VectorXd home_q_;
    mutable Eigen::VectorXd smoothed_q_cmd_;
    std::unordered_map<std::string, Eigen::VectorXd> named_joint_pose_targets_;
    GraspStateMachine::State last_state_ = GraspStateMachine::State::Home;
    bool pregrasp_plan_active_ = false;
    double pregrasp_plan_start_time_ = 0.0;
    double pregrasp_plan_duration_ = 2.0;
    bool pregrasp_refine_active_ = false;
    Eigen::VectorXd pregrasp_start_q_;
    Eigen::VectorXd pregrasp_goal_q_;
    bool descend_plan_active_ = false;
    double descend_plan_start_time_ = 0.0;
    double descend_plan_duration_ = 0.5;
    Eigen::VectorXd descend_start_q_;
    Eigen::VectorXd descend_goal_q_;

    int head_actuator_1_ = -1;
    int head_actuator_2_ = -1;
};

#endif
