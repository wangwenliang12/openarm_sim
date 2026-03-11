#include "robot/robot_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

RobotKinematics::RobotKinematics(const std::string& urdf_path,
                                  const std::string& ee_frame_name,
                                  const std::vector<std::string>& right_arm_joints,
                                  const std::vector<std::string>& left_arm_joints) {
    pinocchio::urdf::buildModel(urdf_path, pin_model_);
    pin_data_ = pinocchio::Data(pin_model_);
    ee_frame_id_ = pin_model_.getFrameId(ee_frame_name);

    q_robot_ = Eigen::VectorXd::Zero(pin_model_.nq);
    dq_robot_ = Eigen::VectorXd::Zero(pin_model_.nv);
    home_q_ = Eigen::VectorXd::Zero(pin_model_.nq);

    // Build JointInfo list from right_arm + left_arm joint names
    // We need to know all joints, so we combine them in the same order
    // as the original code: left arm joints, left fingers, right arm joints, right fingers
    // But the caller provides right_arm_joints and left_arm_joints separately.
    // We'll build joints_ from the union, preserving order.

    auto addJoints = [&](const std::vector<std::string>& names, JointInfo::Group group) {
        for (const auto& name : names) {
            const pinocchio::JointIndex pin_jid = pin_model_.getJointId(name);
            if (pin_jid == 0) {
                throw std::runtime_error("Joint not found in URDF: " + name);
            }
            JointInfo info;
            info.name = name;
            info.pin_q_adr = pin_model_.joints[pin_jid].idx_q();
            info.pin_v_adr = pin_model_.joints[pin_jid].idx_v();
            info.lower_limit = pin_model_.lowerPositionLimit[info.pin_q_adr];
            info.upper_limit = pin_model_.upperPositionLimit[info.pin_q_adr];
            info.is_finger = (name.find("finger") != std::string::npos);
            info.group = group;
            info.home_fixed = (group == JointInfo::Group::LeftArm);
            joints_.push_back(info);

            int idx = static_cast<int>(joints_.size()) - 1;
            if (group == JointInfo::Group::RightArm && !info.is_finger) {
                right_arm_indices_.push_back(idx);
            } else if (group == JointInfo::Group::LeftArm && !info.is_finger) {
                left_arm_indices_.push_back(idx);
            }
        }
    };

    addJoints(left_arm_joints, JointInfo::Group::LeftArm);
    addJoints(right_arm_joints, JointInfo::Group::RightArm);
}

void RobotKinematics::setRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
    q_robot_ = q;
    dq_robot_ = dq;
}

const Eigen::VectorXd& RobotKinematics::currentQ() const {
    return q_robot_;
}

pinocchio::SE3 RobotKinematics::getEndEffectorPose() {
    pinocchio::forwardKinematics(pin_model_, pin_data_, q_robot_, dq_robot_);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);
    return pin_data_.oMf[ee_frame_id_];
}

void RobotKinematics::setHomeQ(const Eigen::VectorXd& q) {
    home_q_ = q;
}

const Eigen::VectorXd& RobotKinematics::homeQ() const {
    return home_q_;
}

const std::vector<JointInfo>& RobotKinematics::joints() const {
    return joints_;
}

int RobotKinematics::nq() const {
    return pin_model_.nq;
}

int RobotKinematics::nv() const {
    return pin_model_.nv;
}

const pinocchio::Model& RobotKinematics::model() const {
    return pin_model_;
}

Eigen::VectorXd RobotKinematics::extractRightArmQ(const Eigen::VectorXd& full_q) const {
    Eigen::VectorXd right_q(rightArmDof());
    for (size_t i = 0; i < right_arm_indices_.size(); ++i) {
        const auto& ji = joints_[static_cast<size_t>(right_arm_indices_[i])];
        right_q[static_cast<int>(i)] = full_q[ji.pin_q_adr];
    }
    return right_q;
}

Eigen::VectorXd RobotKinematics::mergeRightArmQ(const Eigen::VectorXd& right_arm_q,
                                                 const Eigen::VectorXd& base_q) const {
    Eigen::VectorXd merged = base_q;
    for (size_t i = 0; i < right_arm_indices_.size(); ++i) {
        const auto& ji = joints_[static_cast<size_t>(right_arm_indices_[i])];
        merged[ji.pin_q_adr] = right_arm_q[static_cast<int>(i)];
    }
    return merged;
}

std::vector<double> RobotKinematics::rightArmLowerBounds() const {
    std::vector<double> bounds;
    bounds.reserve(right_arm_indices_.size());
    for (const int idx : right_arm_indices_) {
        bounds.push_back(joints_[static_cast<size_t>(idx)].lower_limit);
    }
    return bounds;
}

std::vector<double> RobotKinematics::rightArmUpperBounds() const {
    std::vector<double> bounds;
    bounds.reserve(right_arm_indices_.size());
    for (const int idx : right_arm_indices_) {
        bounds.push_back(joints_[static_cast<size_t>(idx)].upper_limit);
    }
    return bounds;
}

int RobotKinematics::rightArmDof() const {
    return static_cast<int>(right_arm_indices_.size());
}

Eigen::VectorXd RobotKinematics::solveRightArmIK(const Eigen::Vector3d& target_pos,
                                                  const Eigen::Quaterniond& target_quat,
                                                  const Eigen::VectorXd& seed_q) const {
    return solveIK(target_pos, target_quat, seed_q);
}

Eigen::VectorXd RobotKinematics::solveIK(const Eigen::Vector3d& target_pos,
                                          const Eigen::Quaterniond& target_quat,
                                          const Eigen::VectorXd& seed_q) const {
    Eigen::VectorXd q_cmd = seed_q;

    for (const int idx : left_arm_indices_) {
        const auto& ji = joints_[static_cast<size_t>(idx)];
        q_cmd[ji.pin_q_adr] = home_q_[ji.pin_q_adr];
    }

    Eigen::Quaterniond target_quat_normalized = target_quat.normalized();

    for (int iter = 0; iter < 80; ++iter) {
        pinocchio::Data iter_data(pin_model_);
        pinocchio::forwardKinematics(pin_model_, iter_data, q_cmd);
        pinocchio::computeJointJacobians(pin_model_, iter_data, q_cmd);
        pinocchio::updateFramePlacements(pin_model_, iter_data);

        const pinocchio::SE3& current_pose = iter_data.oMf[ee_frame_id_];
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
            pin_model_, iter_data, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

        const int n_right = static_cast<int>(right_arm_indices_.size());
        Eigen::MatrixXd jacobian_right(6, n_right);
        Eigen::VectorXd q_seed_right(n_right);
        Eigen::VectorXd q_current_right(n_right);
        for (size_t i = 0; i < right_arm_indices_.size(); ++i) {
            const auto& ji = joints_[static_cast<size_t>(right_arm_indices_[i])];
            jacobian_right.col(static_cast<int>(i)) = jacobian.col(ji.pin_v_adr);
            q_seed_right[static_cast<int>(i)] = seed_q[ji.pin_q_adr];
            q_current_right[static_cast<int>(i)] = q_cmd[ji.pin_q_adr];
        }

        const double lambda = 5e-3;
        const Eigen::MatrixXd lhs =
            jacobian_right * jacobian_right.transpose() +
            lambda * Eigen::MatrixXd::Identity(6, 6);
        const Eigen::MatrixXd jacobian_pinv =
            jacobian_right.transpose() * lhs.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));

        const Eigen::MatrixXd nullspace =
            Eigen::MatrixXd::Identity(n_right, n_right) - jacobian_pinv * jacobian_right;
        const Eigen::VectorXd seed_bias = 0.15 * (q_seed_right - q_current_right);
        const Eigen::VectorXd delta_q = jacobian_pinv * task_error + nullspace * seed_bias;

        for (size_t i = 0; i < right_arm_indices_.size(); ++i) {
            const auto& ji = joints_[static_cast<size_t>(right_arm_indices_[i])];
            q_cmd[ji.pin_q_adr] += 0.4 * delta_q[static_cast<int>(i)];
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
