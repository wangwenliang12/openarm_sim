#include "task/manipulate_task.hpp"

#include "task/interpolation_motion_planner.hpp"
#include "task/ompl_motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

ManipulateTask::ManipulateTask(const std::string& urdf_path,
                               const std::string& ee_frame_name,
                               const std::vector<std::string>& right_arm_joints,
                               const std::vector<std::string>& left_arm_joints)
    : kinematics_(urdf_path, ee_frame_name, right_arm_joints, left_arm_joints),
      interpolation_planner_(std::make_unique<InterpolationMotionPlanner>(kinematics_)),
      ompl_planner_(
#ifdef OPENARM_HAVE_OMPL
          std::make_unique<OmplMotionPlanner>(kinematics_)
#else
          std::make_unique<InterpolationMotionPlanner>(kinematics_)
#endif
      ) {
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
    if (executor_.hasActiveTrajectory() && executor_.ownerStateId() != next_state_id) {
        executor_.clear();
    }

    if (static_cast<GraspStateMachine::State>(next_state_id) == GraspStateMachine::State::Done) {
        captureHoldPosition();
        return;
    }

    hold_position_active_ = false;
}

bool ManipulateTask::ensureTrajectoryForState(const GraspStateMachine::Command& command,
                                              int state_id, double now) {
    if (!stateRequiresArmPlan(command)) {
        return false;
    }
    if (executor_.hasActiveTrajectory() && executor_.ownerStateId() == state_id) {
        return true;
    }

    const MotionRequest request = buildMotionRequest(command, state_id);
    IMotionPlanner& planner = plannerForRequest(request);
    const char* planner_name =
        (request.goal_type == GoalType::JointGoal) ? "Interpolation" : "OMPL";
    const PlannedTrajectory trajectory = planner.createPlan(request);
    executor_.setTrajectory(trajectory, now);
    std::cout << "[Planner] state=" << state_id
              << " type="
              << ((request.goal_type == GoalType::JointGoal) ? "JointGoal" : "CartesianPoseGoal")
              << " planner=" << planner_name
              << " success=" << (trajectory.valid ? "true" : "false")
              << " points=" << trajectory.points.size()
              << " duration=" << trajectory.duration
              << std::endl;
    return trajectory.valid;
}

Eigen::VectorXd ManipulateTask::sampleJointTargets(double now) const {
    if (!executor_.hasActiveTrajectory()) {
        if (hold_position_active_ && hold_q_.size() == kinematics_.nq()) {
            return hold_q_;
        }
        return kinematics_.currentQ();
    }
    return executor_.sample(now);
}

MotionRequest ManipulateTask::buildMotionRequest(
    const GraspStateMachine::Command& command,
    int state_id) const {
    MotionRequest request;
    request.owner_state_id = state_id;
    request.start_q = kinematics_.currentQ();

    const auto state = static_cast<GraspStateMachine::State>(state_id);
    if (state == GraspStateMachine::State::Descend ||
        state == GraspStateMachine::State::Lift) {
        request.max_joint_velocity = 0.6;
    }

    if (command.mode == GraspStateMachine::CommandMode::JointPose) {
        request.goal_type = GoalType::JointGoal;
        request.joint_goal_q = getNamedPoseTargets(command.joint_pose_name);
    } else {
        request.goal_type = GoalType::CartesianPoseGoal;
        request.target_pos = command.target_pos;
        request.target_quat = command.target_quat;
    }

    return request;
}

bool ManipulateTask::stateRequiresArmPlan(const GraspStateMachine::Command& command) const {
    return command.requires_arm_plan;
}

IMotionPlanner& ManipulateTask::plannerForRequest(const MotionRequest& request) const {
    if (request.goal_type == GoalType::JointGoal) {
        return *interpolation_planner_;
    }
    return *ompl_planner_;
}

void ManipulateTask::captureHoldPosition() {
    hold_q_ = kinematics_.currentQ();
    hold_position_active_ = true;
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
