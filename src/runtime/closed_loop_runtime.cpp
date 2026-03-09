#include "runtime/closed_loop_runtime.hpp"
#include "common/object_pose.hpp"

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

const std::vector<std::string> kLeftArmJoints = {
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7", "openarm_left_finger_joint1", "openarm_left_finger_joint2",
};
const std::vector<std::string> kRightArmJoints = {
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7", "openarm_right_finger_joint1", "openarm_right_finger_joint2",
};
}  // namespace

ClosedLoopRuntime::ClosedLoopRuntime(const mjModel* model)
    : task_("model/urdf/robot/v10.bimanual.generated.urdf", "openarm_right_hand_tcp",
            kRightArmJoints, kLeftArmJoints),
      publisher_(kChannel),
      subscriber_(kChannel),
      state_machine_(kTargetObjectId) {
    if (!publisher_.isReady() || !subscriber_.isReady()) {
        throw std::runtime_error("LCM initialization failed");
    }

    setupJointMaps(model);
    publisher_.initialize(model);
    control_timestep_ = model->opt.timestep;

    // Set hardcoded home_q (matching original)
    Eigen::VectorXd home_q = Eigen::VectorXd::Zero(task_.kinematics().nq());
    if (task_.kinematics().nq() >= 18) {
        home_q[1] = -1.59;
        home_q[3] = 1.88;
        home_q[4] = 1.57;
        home_q[5] = 0.3;

        home_q[10] = 1.59;
        home_q[12] = 1.88;
        home_q[13] = -1.57;
        home_q[14] = -0.3;
    }
    task_.kinematics().setHomeQ(home_q);
}

void ClosedLoopRuntime::initialize(const mjModel* model, const mjData* data) {
    (void)model;
    syncRobotStateToTask(data);

    // Initialize state machine
    const pinocchio::SE3 ee_pose = task_.getEndEffectorPose();
    Eigen::Quaterniond grasp_quat(ee_pose.rotation());
    grasp_quat.normalize();
    state_machine_.initialize(data->time, grasp_quat, ee_pose.translation(),
                              kStateMachineConfigPath);

    // Load named poses after config is loaded
    task_.loadNamedPoses(state_machine_.namedJointPoses());

    // Initialize smoothed command to current state
    task_.setJointAngles(task_.kinematics().currentQ());

    last_state_ = state_machine_.state();
}

void ClosedLoopRuntime::step(const mjModel* model, mjData* data) {
    publisher_.publishTruthPoses(model, data);
    subscriber_.poll();

    syncRobotStateToTask(data);

    const pinocchio::SE3 ee_pose = task_.getEndEffectorPose();
    const Eigen::Vector3d ee_pos = ee_pose.translation();

    // Check named pose reached for JointPose commands
    const bool idle_pose_reached = task_.isNamedPoseReached("idle", kJointPoseReachedTolerance);

    // Get observed object pose
    ObjectPose observed_pose;
    const ObjectPose* observed_pose_ptr =
        subscriber_.getPose(state_machine_.targetObjectId(), &observed_pose)
            ? &observed_pose : nullptr;

    // Update state machine
    GraspStateMachine::Command command = state_machine_.update(
        data->time, ee_pos, observed_pose_ptr, idle_pose_reached);

    // Detect state transition
    const GraspStateMachine::State current_state = state_machine_.state();
    if (current_state != last_state_) {
        // Re-update to get fresh command for new state
        command = state_machine_.update(
            data->time, ee_pos, observed_pose_ptr, idle_pose_reached);

        task_.onStateTransition(
            static_cast<int>(last_state_),
            static_cast<int>(current_state),
            data->time);
    }

    // Compute joint targets
    const int state_id = static_cast<int>(current_state);
    Eigen::VectorXd q_cmd = task_.computeJointTargets(command, state_id, data->time);

    // Smooth
    const ControlLimits limits = controlLimitsForState();
    q_cmd = task_.smoothJointTargets(q_cmd, limits, control_timestep_);

    // Apply controls
    applyRobotGravityCompensation(data);
    writeArmPositionControls(data, q_cmd);
    writeFingerControls(data, command.left_gripper_cmd, command.right_gripper_cmd);
    writeHeadControls(data);
    last_state_ = current_state;
}

void ClosedLoopRuntime::setupJointMaps(const mjModel* model) {
    mj_joints_.clear();
    const auto& joints = task_.kinematics().joints();

    for (const auto& ji : joints) {
        MjJointMap map;
        map.name = ji.name;
        map.pin_q_adr = ji.pin_q_adr;
        map.pin_v_adr = ji.pin_v_adr;

        map.mj_joint_id = mj_name2id(model, mjOBJ_JOINT, ji.name.c_str());
        if (map.mj_joint_id < 0) {
            throw std::runtime_error("MuJoCo joint not found: " + ji.name);
        }
        map.mj_qpos_adr = model->jnt_qposadr[map.mj_joint_id];
        map.mj_dof_adr = model->jnt_dofadr[map.mj_joint_id];
        map.actuator_id = -1;

        mj_joints_.push_back(map);
    }

    setupActuatorIds(model);
}

void ClosedLoopRuntime::setupActuatorIds(const mjModel* model) {
    for (auto& map : mj_joints_) {
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

void ClosedLoopRuntime::syncRobotStateToTask(const mjData* data) {
    const int nq = task_.kinematics().nq();
    const int nv = task_.kinematics().nv();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(nv);

    for (const auto& map : mj_joints_) {
        q[map.pin_q_adr] = data->qpos[map.mj_qpos_adr];
        dq[map.pin_v_adr] = data->qvel[map.mj_dof_adr];
    }

    task_.setRobotState(q, dq);
}

ControlLimits ClosedLoopRuntime::controlLimitsForState() const {
    ControlLimits limits;
    const auto state = state_machine_.state();

    if (state == GraspStateMachine::State::Descend ||
        state == GraspStateMachine::State::Close ||
        state == GraspStateMachine::State::Lift) {
        limits.arm_joint_velocity = kFastCartesianJointVelocity;
        limits.finger_velocity = (state == GraspStateMachine::State::Close)
                                     ? kFastFingerVelocity : kMaxFingerVelocity;
    } else {
        limits.arm_joint_velocity = kMaxJointVelocity;
        limits.finger_velocity = kMaxFingerVelocity;
    }

    return limits;
}

void ClosedLoopRuntime::applyRobotGravityCompensation(mjData* data) const {
    for (const auto& map : mj_joints_) {
        data->qfrc_applied[map.mj_dof_adr] = data->qfrc_bias[map.mj_dof_adr];
    }
}

void ClosedLoopRuntime::writeArmPositionControls(mjData* data, const Eigen::VectorXd& q) const {
    for (const auto& map : mj_joints_) {
        if (map.name.find("joint") == std::string::npos) continue;
        data->ctrl[map.actuator_id] = q[map.pin_q_adr];
    }
}

void ClosedLoopRuntime::writeFingerControls(mjData* data, double left, double right) const {
    const double clamped_left = 0.044 * std::clamp(left, 0.0, 1.0);
    const double clamped_right = 0.044 * std::clamp(right, 0.0, 1.0);

    for (const auto& map : mj_joints_) {
        if (map.name == "openarm_left_finger_joint1" ||
            map.name == "openarm_left_finger_joint2") {
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
