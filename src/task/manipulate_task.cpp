#include "task/manipulate_task.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

ManipulateTask::ManipulateTask(const std::string& urdf_path,
                               const std::string& ee_frame_name,
                               const std::vector<std::string>& right_arm_joints,
                               const std::vector<std::string>& left_arm_joints)
    : kinematics_(urdf_path, ee_frame_name, right_arm_joints, left_arm_joints) {
    smoothed_q_cmd_ = Eigen::VectorXd::Zero(kinematics_.nq());
}

void ManipulateTask::setRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
    kinematics_.setRobotState(q, dq);
}

pinocchio::SE3 ManipulateTask::getEndEffectorPose() {
    return kinematics_.getEndEffectorPose();
}

void ManipulateTask::loadNamedPoses(const NamedPoseMap& poses) {
    named_pose_targets_.clear();
    const auto& joints = kinematics_.joints();
    const auto& model = kinematics_.model();

    for (const auto& [pose_name, joint_map] : poses) {
        Eigen::VectorXd q_pose = kinematics_.currentQ();

        for (const auto& [joint_name, joint_target] : joint_map) {
            bool matched = false;
            for (const auto& ji : joints) {
                if (ji.name != joint_name) continue;
                const double lower = model.lowerPositionLimit[ji.pin_q_adr];
                const double upper = model.upperPositionLimit[ji.pin_q_adr];
                q_pose[ji.pin_q_adr] = std::clamp(joint_target, lower, upper);
                matched = true;
                break;
            }
            if (!matched) {
                throw std::runtime_error("Unknown joint in named pose: " + joint_name);
            }
        }
        named_pose_targets_[pose_name] = q_pose;
    }
}

Eigen::VectorXd ManipulateTask::getNamedPoseTargets(const std::string& name) const {
    const auto it = named_pose_targets_.find(name);
    if (it == named_pose_targets_.end()) {
        throw std::runtime_error("Unknown named joint pose: " + name);
    }
    return it->second;
}

bool ManipulateTask::isNamedPoseReached(const std::string& name, double tol) const {
    const Eigen::VectorXd q_target = getNamedPoseTargets(name);
    return (kinematics_.currentQ() - q_target).cwiseAbs().maxCoeff() < tol;
}

void ManipulateTask::onStateTransition(int prev_state_id, int next_state_id, double now) {
    (void)prev_state_id;
    (void)now;
    // Clear any active plan when state changes — new state will create its own plan
    if (planner_.currentPlan().active &&
        planner_.currentPlan().owner_state_id != next_state_id) {
        planner_.clear();
    }
}

void ManipulateTask::ensurePlanForState(const GraspStateMachine::Command& command,
                                         int state_id, double now) {
    // If we already have an active plan for this state, just manage lifecycle
    if (planner_.isActiveForState(state_id)) {
        const auto mode = planner_.currentPlan().mode;
        if (mode == JointTrajectoryPlanner::PlanMode::InterpolateThenRefine &&
            !planner_.currentPlan().refine_phase &&
            planner_.isInterpolationDone(now)) {
            planner_.enterRefinePhase();
        }
        return;
    }

    // Create a new plan
    const auto mode = planModeForState(state_id);
    const double duration = planDurationForState(state_id);
    const Eigen::VectorXd& start_q = kinematics_.currentQ();
    const Eigen::VectorXd goal_q = kinematics_.solveIK(
        command.target_pos, command.target_quat, start_q);

    planner_.plan(start_q, goal_q, duration, now, mode, state_id,
                  command.target_pos, command.target_quat);
}

Eigen::VectorXd ManipulateTask::computeJointTargets(
    const GraspStateMachine::Command& command,
    int current_state_id,
    double now) {
    if (command.mode == GraspStateMachine::CommandMode::JointPose) {
        return getNamedPoseTargets(command.joint_pose_name);
    }

    // CartesianPose mode — states that use trajectory planning
    const int pregrasp_id = static_cast<int>(GraspStateMachine::State::Pregrasp);
    const int descend_id = static_cast<int>(GraspStateMachine::State::Descend);

    if (current_state_id == pregrasp_id || current_state_id == descend_id) {
        ensurePlanForState(command, current_state_id, now);

        if (planner_.currentPlan().refine_phase) {
            return kinematics_.computeCartesianTargets(
                command.target_pos, command.target_quat);
        }
        return planner_.interpolate(now);
    }

    // Other CartesianPose states: direct IK
    return kinematics_.computeCartesianTargets(
        command.target_pos, command.target_quat);
}

Eigen::VectorXd ManipulateTask::smoothJointTargets(
    const Eigen::VectorXd& q_target,
    const ControlLimits& limits, double dt) const {
    if (smoothed_q_cmd_.size() != q_target.size()) {
        smoothed_q_cmd_ = kinematics_.currentQ();
    }

    const auto& joints = kinematics_.joints();

    for (int i = 0; i < q_target.size(); ++i) {
        double step_limit = limits.arm_joint_velocity * dt;

        for (const auto& ji : joints) {
            if (ji.pin_q_adr != i) continue;
            if (ji.is_finger) {
                step_limit = limits.finger_velocity * dt;
            }
            break;
        }

        const double delta = q_target[i] - smoothed_q_cmd_[i];
        smoothed_q_cmd_[i] += std::clamp(delta, -step_limit, step_limit);
    }

    return smoothed_q_cmd_;
}

void ManipulateTask::setJointAngles(const Eigen::VectorXd& q_target) {
    smoothed_q_cmd_ = q_target;
}

RobotKinematics& ManipulateTask::kinematics() {
    return kinematics_;
}

const RobotKinematics& ManipulateTask::kinematics() const {
    return kinematics_;
}

JointTrajectoryPlanner::PlanMode ManipulateTask::planModeForState(int state_id) const {
    if (state_id == static_cast<int>(GraspStateMachine::State::Pregrasp)) {
        return JointTrajectoryPlanner::PlanMode::InterpolateThenRefine;
    }
    if (state_id == static_cast<int>(GraspStateMachine::State::Descend)) {
        return JointTrajectoryPlanner::PlanMode::InterpolateOnly;
    }
    return JointTrajectoryPlanner::PlanMode::RefineOnly;
}

double ManipulateTask::planDurationForState(int state_id) const {
    if (state_id == static_cast<int>(GraspStateMachine::State::Pregrasp)) {
        return 2.0;
    }
    if (state_id == static_cast<int>(GraspStateMachine::State::Descend)) {
        return 0.5;
    }
    return 1.0;
}
