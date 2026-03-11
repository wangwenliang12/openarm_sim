#ifndef OPENARM_GRASP_STATE_MACHINE_HPP
#define OPENARM_GRASP_STATE_MACHINE_HPP

#include "common/object_pose.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <unordered_map>

class GraspStateMachine {
public:
    enum class CommandMode {
        JointPose,
        CartesianPose,
    };

    enum class State {
        Home,
        Pregrasp,
        Descend,
        Close,
        Lift,
        Done,
    };

    struct Command {
        CommandMode mode = CommandMode::JointPose;
        std::string joint_pose_name;
        Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
        Eigen::Quaterniond target_quat = Eigen::Quaterniond::Identity();
        double left_gripper_cmd = 0.0;
        double right_gripper_cmd = 0.0;
        bool requires_arm_plan = false;
    };

    explicit GraspStateMachine(const std::string& target_object_id);

    void initialize(double now,
                    const Eigen::Vector3d& current_pos,
                    const std::string& config_path);
                    
    Command update(double now,
                   const Eigen::Vector3d& current_pos,
                   const ObjectPose* observed_pose,
                   bool active_joint_pose_reached);

    State state() const;
    const std::string& targetObjectId() const;
    const std::unordered_map<std::string, std::unordered_map<std::string, double>>&
    namedJointPoses() const;

private:
    struct Config {
        bool home_only_mode = false;
        double home_duration_sec = 2.0;
        double close_duration_sec = 1.2;
        double pregrasp_height = 0.02;
        double descend_height = 0.005;
        double lift_height = 0.22;
        double pregrasp_tolerance = 0.02;
        double descend_tolerance = 0.020;
        double lift_tolerance = 0.03;
        std::unordered_map<std::string, std::unordered_map<std::string, double>> named_joint_poses;
    };

    void loadConfig(const std::string& config_path);
    void transitionTo(State next, double now);
    static const char* stateName(State state);
    bool shouldLeaveHome(bool active_joint_pose_reached, double now) const;

    std::string target_object_id_;
    State state_ = State::Home;
    double state_start_time_ = 0.0;
    ObjectPose object_pose_;
    ObjectPose grasp_target_pose_;
    Config config_;
    Eigen::Vector3d pregrasp_start_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d lift_start_pos_ = Eigen::Vector3d::Zero();
    bool pregrasp_started_ = false;
    bool grasp_target_locked_ = false;

    Command handleHome(double now, const Eigen::Vector3d& current_pos, bool active_joint_pose_reached);
    Command handlePregrasp(double now, const Eigen::Vector3d& current_pos);
    Command handleDescend(double now, const Eigen::Vector3d& current_pos);
    Command handleClose(double now, const Eigen::Vector3d& current_pos);
    Command handleLift(double now, const Eigen::Vector3d& current_pos);
    Command handleDone(double now, const Eigen::Vector3d& current_pos);
};

#endif
