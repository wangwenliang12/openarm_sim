#include "runtime/closed_loop_runtime.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr const char* kChannel = "OPENARM_OBJECT_POSES";
constexpr const char* kTargetObjectId = "red_block";
constexpr const char* kStateMachineConfigPath = "config/grasp_state_machine.json";
constexpr double kJointPoseReachedTolerance = 0.05;
constexpr double kMaxJointVelocity = 0.25;
constexpr double kFastCartesianJointVelocity = 0.6;
constexpr double kMaxFingerVelocity = 0.02;
constexpr double kFastFingerVelocity = 0.05;
}

ClosedLoopRuntime::ClosedLoopRuntime(const mjModel* model)
    : publisher_(kChannel),
      subscriber_(kChannel),
      state_machine_(kTargetObjectId) {
    pinocchio::urdf::buildModel("model/urdf/robot/v10.bimanual.generated.urdf", pin_model_);
    pin_data_ = pinocchio::Data(pin_model_);
    active_ee_frame_id_ = pin_model_.getFrameId("openarm_right_hand_tcp");

    if (!publisher_.isReady() || !subscriber_.isReady()) {
        throw std::runtime_error("LCM initialization failed");
    }

    setupJointMaps(model);
    setupActuatorIds(model);
    publisher_.initialize(model);
    control_timestep_ = model->opt.timestep;

    q_robot_ = Eigen::VectorXd::Zero(pin_model_.nq);
    dq_robot_ = Eigen::VectorXd::Zero(pin_model_.nv);
    home_q_ = Eigen::VectorXd::Zero(pin_model_.nq);
    smoothed_q_cmd_ = Eigen::VectorXd::Zero(pin_model_.nq);
    pregrasp_start_q_ = Eigen::VectorXd::Zero(pin_model_.nq);
    pregrasp_goal_q_ = Eigen::VectorXd::Zero(pin_model_.nq);
    descend_start_q_ = Eigen::VectorXd::Zero(pin_model_.nq);
    descend_goal_q_ = Eigen::VectorXd::Zero(pin_model_.nq);

    if (pin_model_.nq >= 18) {
        home_q_[1] = -1.59;
        home_q_[3] = 1.88;
        home_q_[4] = 1.57;
        home_q_[5] = 0.3;

        home_q_[10] = 1.59;
        home_q_[12] = 1.88;
        home_q_[13] = -1.57;
        home_q_[14] = -0.3;
    }
}

void ClosedLoopRuntime::initialize(const mjModel* model, const mjData* data) {
    (void)model;
    updateRobotState(data);
    const pinocchio::SE3 pose = getCurrentPose();
    Eigen::Quaterniond grasp_quat(pose.rotation());
    grasp_quat.normalize();
    state_machine_.initialize(data->time, grasp_quat, pose.translation(), kStateMachineConfigPath);
    cacheNamedJointPoses();
    smoothed_q_cmd_ = q_robot_;
    pregrasp_start_q_ = q_robot_;
    pregrasp_goal_q_ = q_robot_;
    pregrasp_refine_active_ = false;
    descend_start_q_ = q_robot_;
    descend_goal_q_ = q_robot_;
    descend_plan_active_ = false;
    last_state_ = state_machine_.state();
}

void ClosedLoopRuntime::step(const mjModel* model, mjData* data) {
    publisher_.publishTruthPoses(model, data);
    subscriber_.poll();

    updateRobotState(data);
    const pinocchio::SE3 current_pose = getCurrentPose();

    ObjectPose observed_pose;
    const ObjectPose* observed_pose_ptr =
        subscriber_.getPose(state_machine_.targetObjectId(), &observed_pose) ? &observed_pose : nullptr;
    const bool idle_pose_reached = isNamedJointPoseReached("idle", kJointPoseReachedTolerance);

    GraspStateMachine::Command command =
        state_machine_.update(data->time, current_pose.translation(), observed_pose_ptr, idle_pose_reached);

    // If the state machine transitioned inside update(), refresh the command so planning logic
    // uses the new state's target rather than the previous state's final command.
    if (state_machine_.state() != last_state_) {
        command = state_machine_.update(
            data->time, current_pose.translation(), observed_pose_ptr, idle_pose_reached);
    }

    updatePregraspPlanIfNeeded(command, current_pose, data->time);
    updateDescendPlanIfNeeded(command, data->time);

    Eigen::VectorXd q_cmd = home_q_;
    if (command.mode == GraspStateMachine::CommandMode::JointPose) {
        q_cmd = getNamedJointPoseTargets(command.joint_pose_name);
    } else if (state_machine_.state() == GraspStateMachine::State::Pregrasp && pregrasp_plan_active_) {
        if (pregrasp_refine_active_) {
            q_cmd = computeTaskJointTargets(command.target_pos, command.target_quat);
        } else {
            q_cmd = interpolatePregraspJointTargets(data->time);
        }
    } else if (state_machine_.state() == GraspStateMachine::State::Descend && descend_plan_active_) {
        q_cmd = interpolateDescendJointTargets(data->time);
    } else {
        q_cmd = computeTaskJointTargets(command.target_pos, command.target_quat);
    }

    q_cmd = smoothJointTargets(q_cmd);

    applyRobotGravityCompensation(data);
    writeArmPositionControls(data, q_cmd);
    writeFingerControls(data, command.left_gripper_cmd, command.right_gripper_cmd);
    writeHeadControls(data);
    last_state_ = state_machine_.state();
}

void ClosedLoopRuntime::setupJointMaps(const mjModel* model) {
    const std::vector<std::string> joint_names = {
        "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
        "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
        "openarm_left_joint7", "openarm_left_finger_joint1", "openarm_left_finger_joint2",
        "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
        "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
        "openarm_right_joint7", "openarm_right_finger_joint1", "openarm_right_finger_joint2",
    };

    for (const auto& name : joint_names) {
        const int mj_joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
        const pinocchio::JointIndex pin_joint_id = pin_model_.getJointId(name);
        if (mj_joint_id < 0 || pin_joint_id == 0) {
            throw std::runtime_error("Joint mapping failed for " + name);
        }

        RobotJointMap map;
        map.name = name;
        map.mj_joint_id = mj_joint_id;
        map.mj_qpos_adr = model->jnt_qposadr[mj_joint_id];
        map.mj_dof_adr = model->jnt_dofadr[mj_joint_id];
        map.pin_q_adr = pin_model_.joints[pin_joint_id].idx_q();
        map.pin_v_adr = pin_model_.joints[pin_joint_id].idx_v();
        robot_joints_.push_back(map);

        if (name.find("openarm_left_joint") != std::string::npos) {
            left_arm_joint_indices_.push_back(static_cast<int>(robot_joints_.size()) - 1);
        } else if (name.find("openarm_right_joint") != std::string::npos) {
            right_arm_joint_indices_.push_back(static_cast<int>(robot_joints_.size()) - 1);
        }
    }
}

void ClosedLoopRuntime::setupActuatorIds(const mjModel* model) {
    for (auto& map : robot_joints_) {
        std::string actuator_name;
        if (map.name.find("left_joint") != std::string::npos) {
            actuator_name = "left_joint" + map.name.substr(map.name.size() - 1) + "_ctrl";
        } else if (map.name.find("right_joint") != std::string::npos) {
            actuator_name = "right_joint" + map.name.substr(map.name.size() - 1) + "_ctrl";
        } else if (map.name == "openarm_left_finger_joint1") {
            actuator_name = "left_finger1_ctrl";
        } else if (map.name == "openarm_left_finger_joint2") {
            actuator_name = "left_finger2_ctrl";
        } else if (map.name == "openarm_right_finger_joint1") {
            actuator_name = "right_finger1_ctrl";
        } else if (map.name == "openarm_right_finger_joint2") {
            actuator_name = "right_finger2_ctrl";
        }

        map.actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
        if (map.actuator_id < 0) {
            throw std::runtime_error("Actuator mapping failed for " + actuator_name);
        }
    }

    head_actuator_1_ = mj_name2id(model, mjOBJ_ACTUATOR, "head_joint1_ctrl");
    head_actuator_2_ = mj_name2id(model, mjOBJ_ACTUATOR, "head_joint2_ctrl");
}

void ClosedLoopRuntime::updateRobotState(const mjData* data) {
    q_robot_.setZero();
    dq_robot_.setZero();
    for (const auto& map : robot_joints_) {
        q_robot_[map.pin_q_adr] = data->qpos[map.mj_qpos_adr];
        dq_robot_[map.pin_v_adr] = data->qvel[map.mj_dof_adr];
    }
}

void ClosedLoopRuntime::cacheNamedJointPoses() {
    named_joint_pose_targets_.clear();

    for (const auto& [pose_name, joints] : state_machine_.namedJointPoses()) {
        Eigen::VectorXd q_pose = q_robot_;

        for (const auto& [joint_name, joint_target] : joints) {
            bool matched = false;
            for (const auto& map : robot_joints_) {
                if (map.name != joint_name) {
                    continue;
                }
                const double lower = pin_model_.lowerPositionLimit[map.pin_q_adr];
                const double upper = pin_model_.upperPositionLimit[map.pin_q_adr];
                q_pose[map.pin_q_adr] = std::clamp(joint_target, lower, upper);
                matched = true;
                break;
            }

            if (!matched) {
                throw std::runtime_error("Unknown joint in grasp state machine config: " + joint_name);
            }
        }

        named_joint_pose_targets_[pose_name] = q_pose;
    }
}

Eigen::VectorXd ClosedLoopRuntime::getNamedJointPoseTargets(const std::string& pose_name) const {
    const auto it = named_joint_pose_targets_.find(pose_name);
    if (it == named_joint_pose_targets_.end()) {
        throw std::runtime_error("Unknown named joint pose: " + pose_name);
    }
    return it->second;
}

bool ClosedLoopRuntime::isNamedJointPoseReached(const std::string& pose_name, double tolerance) const {
    const Eigen::VectorXd q_target = getNamedJointPoseTargets(pose_name);
    return (q_robot_ - q_target).cwiseAbs().maxCoeff() < tolerance;
}

Eigen::VectorXd ClosedLoopRuntime::smoothJointTargets(const Eigen::VectorXd& q_target) const {
    if (smoothed_q_cmd_.size() != q_target.size()) {
        smoothed_q_cmd_ = q_robot_;
    }

    const double joint_velocity_limit = jointVelocityLimitForState();
    const double finger_velocity_limit = fingerVelocityLimitForState();

    for (int i = 0; i < q_target.size(); ++i) {
        double step_limit = joint_velocity_limit * control_timestep_;

        for (const auto& map : robot_joints_) {
            if (map.pin_q_adr != i) {
                continue;
            }
            if (map.name.find("finger") != std::string::npos) {
                step_limit = finger_velocity_limit * control_timestep_;
            }
            break;
        }

        const double delta = q_target[i] - smoothed_q_cmd_[i];
        smoothed_q_cmd_[i] += std::clamp(delta, -step_limit, step_limit);
    }

    return smoothed_q_cmd_;
}

double ClosedLoopRuntime::jointVelocityLimitForState() const {
    switch (state_machine_.state()) {
    case GraspStateMachine::State::Descend:
    case GraspStateMachine::State::Close:
    case GraspStateMachine::State::Lift:
        return kFastCartesianJointVelocity;
    case GraspStateMachine::State::Pregrasp:
    case GraspStateMachine::State::Home:
    case GraspStateMachine::State::Done:
        return kMaxJointVelocity;
    }
    return kMaxJointVelocity;
}

double ClosedLoopRuntime::fingerVelocityLimitForState() const {
    switch (state_machine_.state()) {
    case GraspStateMachine::State::Close:
        return kFastFingerVelocity;
    case GraspStateMachine::State::Descend:
    case GraspStateMachine::State::Lift:
    case GraspStateMachine::State::Pregrasp:
    case GraspStateMachine::State::Home:
    case GraspStateMachine::State::Done:
        return kMaxFingerVelocity;
    }
    return kMaxFingerVelocity;
}

void ClosedLoopRuntime::updatePregraspPlanIfNeeded(const GraspStateMachine::Command& command,
                                                   const pinocchio::SE3& current_pose,
                                                   double now) {
    if (state_machine_.state() != GraspStateMachine::State::Pregrasp ||
        command.mode != GraspStateMachine::CommandMode::CartesianPose) {
        pregrasp_plan_active_ = false;
        pregrasp_refine_active_ = false;
        return;
    }

    if (last_state_ == GraspStateMachine::State::Pregrasp && pregrasp_plan_active_) {
        const double elapsed = now - pregrasp_plan_start_time_;
        if (!pregrasp_refine_active_ && elapsed >= pregrasp_plan_duration_) {
            pregrasp_refine_active_ = true;
        }
        return;
    }

    pregrasp_plan_active_ = true;
    pregrasp_refine_active_ = false;
    pregrasp_plan_start_time_ = now;
    pregrasp_start_q_ = q_robot_;
    pregrasp_plan_duration_ = 2.0;
    pregrasp_goal_q_ = solveRightArmIkGoal(command.target_pos, command.target_quat, pregrasp_start_q_);
    (void)current_pose;
}

Eigen::VectorXd ClosedLoopRuntime::interpolatePregraspJointTargets(double now) const {
    const double alpha = std::clamp(
        (now - pregrasp_plan_start_time_) / std::max(1e-3, pregrasp_plan_duration_), 0.0, 1.0);
    const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
    return pregrasp_start_q_ + smooth_alpha * (pregrasp_goal_q_ - pregrasp_start_q_);
}

void ClosedLoopRuntime::updateDescendPlanIfNeeded(const GraspStateMachine::Command& command, double now) {
    if (state_machine_.state() != GraspStateMachine::State::Descend ||
        command.mode != GraspStateMachine::CommandMode::CartesianPose) {
        descend_plan_active_ = false;
        return;
    }

    if (last_state_ == GraspStateMachine::State::Descend && descend_plan_active_) {
        return;
    }

    descend_plan_active_ = true;
    descend_plan_start_time_ = now;
    descend_start_q_ = q_robot_;
    descend_plan_duration_ = 0.5;
    descend_goal_q_ = solveRightArmIkGoal(command.target_pos, command.target_quat, descend_start_q_);
}

Eigen::VectorXd ClosedLoopRuntime::interpolateDescendJointTargets(double now) const {
    const double alpha = std::clamp(
        (now - descend_plan_start_time_) / std::max(1e-3, descend_plan_duration_), 0.0, 1.0);
    const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
    return descend_start_q_ + smooth_alpha * (descend_goal_q_ - descend_start_q_);
}

Eigen::VectorXd ClosedLoopRuntime::solveRightArmIkGoal(const Eigen::Vector3d& target_pos,
                                                       const Eigen::Quaterniond& target_quat,
                                                       const Eigen::VectorXd& seed_q) const {
    Eigen::VectorXd q_cmd = seed_q;

    for (const int idx : left_arm_joint_indices_) {
        const auto& map = robot_joints_[static_cast<size_t>(idx)];
        q_cmd[map.pin_q_adr] = home_q_[map.pin_q_adr];
    }

    Eigen::Quaterniond target_quat_normalized = target_quat.normalized();

    for (int iter = 0; iter < 80; ++iter) {
        pinocchio::Data iter_data(pin_model_);
        pinocchio::forwardKinematics(pin_model_, iter_data, q_cmd);
        pinocchio::computeJointJacobians(pin_model_, iter_data, q_cmd);
        pinocchio::updateFramePlacements(pin_model_, iter_data);

        const pinocchio::SE3& current_pose = iter_data.oMf[active_ee_frame_id_];
        const Eigen::Vector3d position_error = target_pos - current_pose.translation();

        Eigen::Quaterniond current_quat(current_pose.rotation());
        current_quat.normalize();
        Eigen::Quaterniond quat_error = target_quat_normalized * current_quat.conjugate();
        if (quat_error.w() < 0.0) {
            quat_error.coeffs() *= -1.0;
        }

        Eigen::AngleAxisd angle_axis(quat_error);
        Eigen::Matrix<double, 6, 1> task_error = Eigen::Matrix<double, 6, 1>::Zero();
        task_error.head<3>() = position_error;
        task_error.tail<3>() = 0.35 * angle_axis.axis() * angle_axis.angle();

        if (task_error.head<3>().norm() < 0.002 && task_error.tail<3>().norm() < 0.03) {
            break;
        }

        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, pin_model_.nv);
        pinocchio::getFrameJacobian(
            pin_model_, iter_data, active_ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

        Eigen::MatrixXd jacobian_right(6, static_cast<int>(right_arm_joint_indices_.size()));
        Eigen::VectorXd q_seed_right(static_cast<int>(right_arm_joint_indices_.size()));
        Eigen::VectorXd q_current_right(static_cast<int>(right_arm_joint_indices_.size()));
        for (size_t i = 0; i < right_arm_joint_indices_.size(); ++i) {
            const auto& map = robot_joints_[static_cast<size_t>(right_arm_joint_indices_[i])];
            jacobian_right.col(static_cast<int>(i)) = jacobian.col(map.pin_v_adr);
            q_seed_right[static_cast<int>(i)] = seed_q[map.pin_q_adr];
            q_current_right[static_cast<int>(i)] = q_cmd[map.pin_q_adr];
        }

        const double lambda = 5e-3;
        const Eigen::MatrixXd lhs =
            jacobian_right * jacobian_right.transpose() +
            lambda * Eigen::MatrixXd::Identity(6, 6);
        const Eigen::MatrixXd jacobian_pinv =
            jacobian_right.transpose() * lhs.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));

        const Eigen::MatrixXd nullspace =
            Eigen::MatrixXd::Identity(jacobian_right.cols(), jacobian_right.cols()) -
            jacobian_pinv * jacobian_right;
        const Eigen::VectorXd seed_bias = 0.15 * (q_seed_right - q_current_right);
        const Eigen::VectorXd delta_q = jacobian_pinv * task_error + nullspace * seed_bias;

        for (size_t i = 0; i < right_arm_joint_indices_.size(); ++i) {
            const auto& map = robot_joints_[static_cast<size_t>(right_arm_joint_indices_[i])];
            q_cmd[map.pin_q_adr] += 0.4 * delta_q[static_cast<int>(i)];
        }

        for (int q_idx = 0; q_idx < q_cmd.size(); ++q_idx) {
            const double lower = pin_model_.lowerPositionLimit[q_idx];
            const double upper = pin_model_.upperPositionLimit[q_idx];
            if (lower <= upper) {
                q_cmd[q_idx] = std::clamp(q_cmd[q_idx], lower, upper);
            }
        }
    }

    return q_cmd;
}

void ClosedLoopRuntime::applyRobotGravityCompensation(mjData* data) const {
    for (const auto& map : robot_joints_) {
        data->qfrc_applied[map.mj_dof_adr] = data->qfrc_bias[map.mj_dof_adr];
    }
}

pinocchio::SE3 ClosedLoopRuntime::getCurrentPose() {
    pinocchio::forwardKinematics(pin_model_, pin_data_, q_robot_, dq_robot_);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);
    return pin_data_.oMf[active_ee_frame_id_];
}

Eigen::VectorXd ClosedLoopRuntime::computeTaskJointTargets(const Eigen::Vector3d& target_pos,
                                                           const Eigen::Quaterniond& target_quat) const {
    return solveRightArmIkGoal(target_pos, target_quat, q_robot_);
}

void ClosedLoopRuntime::writeArmPositionControls(mjData* data, const Eigen::VectorXd& q_cmd) const {
    for (const auto& map : robot_joints_) {
        if (map.name.find("joint") == std::string::npos) {
            continue;
        }
        data->ctrl[map.actuator_id] = q_cmd[map.pin_q_adr];
    }
}

void ClosedLoopRuntime::writeFingerControls(mjData* data, double left_cmd, double right_cmd) const {
    const double clamped_left = 0.044 * std::clamp(left_cmd, 0.0, 1.0);
    const double clamped_right = 0.044 * std::clamp(right_cmd, 0.0, 1.0);

    for (const auto& map : robot_joints_) {
        if (map.name == "openarm_left_finger_joint1" || map.name == "openarm_left_finger_joint2") {
            data->ctrl[map.actuator_id] = clamped_left;
        } else if (map.name == "openarm_right_finger_joint1" ||
                   map.name == "openarm_right_finger_joint2") {
            data->ctrl[map.actuator_id] = clamped_right;
        }
    }
}

void ClosedLoopRuntime::writeHeadControls(mjData* data) const {
    if (head_actuator_1_ >= 0) {
        data->ctrl[head_actuator_1_] = 0.0;
    }
    if (head_actuator_2_ >= 0) {
        data->ctrl[head_actuator_2_] = 0.0;
    }
}
