#include "state_machine/grasp_state_machine.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void loadJointGroup(const nlohmann::json& group,std::unordered_map<std::string, double>* joint_positions) {
    if (!group.is_object()) {
        throw std::runtime_error("joint group must be a JSON object");
    }

    for (auto it = group.begin(); it != group.end(); ++it) {
        if (!it.value().is_number()) {
            throw std::runtime_error("Joint target must be numeric for " + it.key());
        }
        (*joint_positions)[it.key()] = it.value().get<double>();
    }
}

Eigen::Quaterniond makeTopDownGraspQuat(const Eigen::Quaterniond& object_quat) {
    constexpr double kPi = 3.14159265358979323846;
    const Eigen::Vector3d object_x = object_quat.normalized() * Eigen::Vector3d::UnitX();
    const double yaw = std::atan2(object_x.y(), object_x.x());
    const Eigen::AngleAxisd yaw_rotation(yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd tool_alignment(kPi, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond grasp_quat(yaw_rotation * tool_alignment);
    grasp_quat.normalize();
    return grasp_quat;
}
}  // namespace

GraspStateMachine::GraspStateMachine(const std::string& target_object_id)
    : target_object_id_(target_object_id) {}

void GraspStateMachine::initialize(double now,
                                   const Eigen::Quaterniond& grasp_quat,
                                   const Eigen::Vector3d& current_pos,
                                   const std::string& config_path) {
    loadConfig(config_path);

    grasp_quat_ = grasp_quat;
    grasp_quat_.normalize();
    state_ = State::Home;
    state_start_time_ = now;
    object_pose_.position = current_pos;
    grasp_target_pose_ = object_pose_;
    pregrasp_start_pos_ = current_pos;
    pregrasp_started_ = false;
    grasp_target_locked_ = false;
}

GraspStateMachine::Command GraspStateMachine::update(double now,
                                                     const Eigen::Vector3d& current_pos,
                                                     const ObjectPose* observed_pose,
                                                     bool active_joint_pose_reached) {
    if (observed_pose != nullptr) {
        object_pose_ = *observed_pose;
        if (!grasp_target_locked_) {
            grasp_target_pose_ = *observed_pose;
        }
    }

    switch (state_) {
    case State::Home:
        return handleHome(now, current_pos, active_joint_pose_reached);
    case State::Pregrasp:
        return handlePregrasp(now, current_pos);
    case State::Descend:
        return handleDescend(now, current_pos);
    case State::Close:
        return handleClose(now, current_pos);
    case State::Lift:
        return handleLift(now, current_pos);
    case State::Done:
        return handleDone(now, current_pos);
    }

    throw std::runtime_error("Unknown grasp state");
}

GraspStateMachine::State GraspStateMachine::state() const {
    return state_;
}

const std::string& GraspStateMachine::targetObjectId() const {
    return target_object_id_;
}

const std::unordered_map<std::string, std::unordered_map<std::string, double>>&
GraspStateMachine::namedJointPoses() const {
    return config_.named_joint_poses;
}

void GraspStateMachine::loadConfig(const std::string& config_path) {
    config_ = Config{};

    if (config_path.empty()) {
        return;
    }

    std::ifstream input(config_path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open grasp state machine config: " + config_path);
    }

    nlohmann::json root;
    input >> root;

    if (root.contains("timing")) {
        const auto& timing = root.at("timing");
        config_.home_only_mode = timing.value("home_only_mode", config_.home_only_mode);
        config_.home_duration_sec = timing.value("home_duration_sec", config_.home_duration_sec);
        config_.close_duration_sec = timing.value("close_duration_sec", config_.close_duration_sec);
        config_.pregrasp_move_duration_sec =
            timing.value("pregrasp_move_duration_sec", config_.pregrasp_move_duration_sec);
    }

    if (root.contains("cartesian_targets")) {
        const auto& targets = root.at("cartesian_targets");
        config_.pregrasp_height = targets.value("pregrasp_height", config_.pregrasp_height);
        config_.descend_height = targets.value("descend_height", config_.descend_height);
        config_.lift_height = targets.value("lift_height", config_.lift_height);
        config_.pregrasp_tolerance = targets.value("pregrasp_tolerance", config_.pregrasp_tolerance);
        config_.descend_tolerance = targets.value("descend_tolerance", config_.descend_tolerance);
        config_.lift_tolerance = targets.value("lift_tolerance", config_.lift_tolerance);
    }

    if (root.contains("poses")) {
        const auto& poses = root.at("poses");
        if (!poses.is_object()) {
            throw std::runtime_error("poses must be a JSON object");
        }

        for (auto pose_it = poses.begin(); pose_it != poses.end(); ++pose_it) {
            std::unordered_map<std::string, double> joint_pose;
            const auto& pose_value = pose_it.value();
            if (!pose_value.is_object()) {
                throw std::runtime_error("pose entry must be a JSON object: " + pose_it.key());
            }

            if (pose_value.contains("left_arm")) {
                loadJointGroup(pose_value.at("left_arm"), &joint_pose);
            }
            if (pose_value.contains("right_arm")) {
                loadJointGroup(pose_value.at("right_arm"), &joint_pose);
            }

            config_.named_joint_poses[pose_it.key()] = std::move(joint_pose);
        }
    }
}

void GraspStateMachine::transitionTo(State next, double now) {
    if (state_ == next) {
        return;
    }
    state_ = next;
    state_start_time_ = now;
    if (state_ == State::Pregrasp) {
        pregrasp_started_ = false;
        grasp_target_locked_ = false;
    } else if (state_ == State::Descend) {
        grasp_target_pose_ = object_pose_;
        grasp_target_locked_ = true;
    } else if (state_ == State::Home) {
        grasp_target_locked_ = false;
    }
    std::cout << "[GraspStateMachine] transition -> " << stateName(state_) << std::endl;
}

const char* GraspStateMachine::stateName(State state) {
    switch (state) {
    case State::Home:
        return "Home";
    case State::Pregrasp:
        return "Pregrasp";
    case State::Descend:
        return "Descend";
    case State::Close:
        return "Close";
    case State::Lift:
        return "Lift";
    case State::Done:
        return "Done";
    }
    return "Unknown";
}

GraspStateMachine::Command GraspStateMachine::handleHome(double now,
                                                         const Eigen::Vector3d& current_pos,
                                                         bool active_joint_pose_reached) {
    (void)current_pos;
    Command cmd;
    cmd.mode = CommandMode::JointPose;
    cmd.joint_pose_name = "idle";
    cmd.left_gripper_cmd = 1.0;
    cmd.right_gripper_cmd = 1.0;

    if (shouldLeaveHome(active_joint_pose_reached, now)) {
        transitionTo(State::Pregrasp, now);
    }
    return cmd;
}

bool GraspStateMachine::shouldLeaveHome(bool active_joint_pose_reached, double now) const {
    if (config_.home_only_mode) {
        return false;
    }
    return active_joint_pose_reached && now - state_start_time_ > config_.home_duration_sec;
}

GraspStateMachine::Command GraspStateMachine::handlePregrasp(double now,const Eigen::Vector3d& current_pos) {
    if (!pregrasp_started_) {
        pregrasp_start_pos_ = current_pos;
        pregrasp_started_ = true;
    }

    Command cmd;
    cmd.mode = CommandMode::CartesianPose;
    cmd.target_quat = makeTopDownGraspQuat(grasp_target_pose_.quaternion);
    cmd.left_gripper_cmd = 1.0;
    cmd.right_gripper_cmd = 1.0;

    const Eigen::Vector3d goal_pos =
        grasp_target_pose_.position + Eigen::Vector3d(0.0, 0.0, config_.pregrasp_height);
    cmd.target_pos = goal_pos;
    const double goal_error = (current_pos - goal_pos).norm();

    if (goal_error < config_.pregrasp_tolerance) {
        transitionTo(State::Descend, now);
    }

    return cmd;
}

GraspStateMachine::Command GraspStateMachine::handleDescend(double now,const Eigen::Vector3d& current_pos) {
    Command cmd;
    cmd.mode = CommandMode::CartesianPose;
    cmd.target_quat = makeTopDownGraspQuat(grasp_target_pose_.quaternion);
    cmd.target_pos = grasp_target_pose_.position + Eigen::Vector3d(0.0, 0.0, config_.descend_height);
    cmd.left_gripper_cmd = 1.0;
    cmd.right_gripper_cmd = 1.0;
    const double descend_error = (current_pos - cmd.target_pos).norm();

    if (descend_error < config_.descend_tolerance) {
        transitionTo(State::Close, now);
    }
    return cmd;
}

GraspStateMachine::Command GraspStateMachine::handleClose(double now,const Eigen::Vector3d& current_pos) {
    Command cmd;
    cmd.mode = CommandMode::CartesianPose;
    cmd.target_quat = makeTopDownGraspQuat(grasp_target_pose_.quaternion);
    cmd.target_pos = grasp_target_pose_.position + Eigen::Vector3d(0.0, 0.0, config_.descend_height);
    cmd.left_gripper_cmd = 1.0;
    cmd.right_gripper_cmd = 0.0;

    if (now - state_start_time_ > config_.close_duration_sec) {
        transitionTo(State::Lift, now);
    }
    return cmd;
}

GraspStateMachine::Command GraspStateMachine::handleLift(double now,const Eigen::Vector3d& current_pos) {
    Command cmd;
    cmd.mode = CommandMode::CartesianPose;
    cmd.target_quat = makeTopDownGraspQuat(grasp_target_pose_.quaternion);
    cmd.target_pos = grasp_target_pose_.position + Eigen::Vector3d(0.0, 0.0, config_.lift_height);
    cmd.left_gripper_cmd = 1.0;
    cmd.right_gripper_cmd = 0.0;

    if ((current_pos - cmd.target_pos).norm() < config_.lift_tolerance) {
        transitionTo(State::Done, now);
    }
    return cmd;
}

GraspStateMachine::Command GraspStateMachine::handleDone(double now,const Eigen::Vector3d& current_pos) {
    (void)now;
    (void)current_pos;
    Command cmd;
    cmd.mode = CommandMode::CartesianPose;
    cmd.target_quat = makeTopDownGraspQuat(grasp_target_pose_.quaternion);
    cmd.target_pos = grasp_target_pose_.position + Eigen::Vector3d(0.0, 0.0, config_.lift_height);
    cmd.left_gripper_cmd = 1.0;
    cmd.right_gripper_cmd = 0.0;
    return cmd;
}
