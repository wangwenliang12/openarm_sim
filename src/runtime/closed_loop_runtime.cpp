#include "runtime/closed_loop_runtime.hpp"
#include "common/object_pose.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
constexpr const char* kChannel = "OPENARM_OBJECT_POSES";
constexpr const char* kTargetObjectId = "red_block";
constexpr const char* kStateMachineConfigPath = "config/grasp_state_machine.json";
constexpr double kJointPoseReachedTolerance = 0.05;
constexpr double kMaxJointVelocity = 0.20;
constexpr double kFastCartesianJointVelocity = 0.20;
constexpr double kMaxFingerVelocity = 0.15;
constexpr double kFastFingerVelocity = 0.20;

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

const char* stateName(GraspStateMachine::State state) {
    switch (state) {
    case GraspStateMachine::State::Home:
        return "Home";
    case GraspStateMachine::State::Pregrasp:
        return "Pregrasp";
    case GraspStateMachine::State::Descend:
        return "Descend";
    case GraspStateMachine::State::Close:
        return "Close";
    case GraspStateMachine::State::Lift:
        return "Lift";
    case GraspStateMachine::State::Done:
        return "Done";
    }
    return "Unknown";
}
}  // namespace

ClosedLoopRuntime::ClosedLoopRuntime(const mjModel* model)//在初始化closedloopruntime之前将manioulatetask初始化
    : task_("model/urdf/robot/v10.bimanual.generated.urdf", "openarm_right_hand_tcp",
            kRightArmJoints, kLeftArmJoints),
      publisher_(kChannel),
      subscriber_(kChannel),
      state_machine_(kTargetObjectId) {
    if (!publisher_.isReady() || !subscriber_.isReady()) {
        throw std::runtime_error("LCM initialization failed");
    }
    // 建立mujoco与pinocchio之间的关节索引表
    setupJointMaps(model);
    publisher_.initialize(model);
    // 读取设置的仿真时间步长
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
    // MuJoCo 当前状态同步到 `ManipulateTask`
    syncRobotStateToTask(data);

    // Initialize state machine
    const pinocchio::SE3 ee_pose = task_.getEndEffectorPose();
    state_machine_.initialize(data->time, ee_pose.translation(), kStateMachineConfigPath);

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
    const ObjectPose* observed_pose_ptr = subscriber_.getPose(state_machine_.targetObjectId(), &observed_pose)? 
                                                                &observed_pose : nullptr;

    // Update state machine
    GraspStateMachine::Command command = state_machine_.update(data->time, ee_pos, observed_pose_ptr, idle_pose_reached);

    // Detect state transition
    const GraspStateMachine::State current_state = state_machine_.state();
    if (current_state != last_state_) {
        // Re-update to get fresh command for new state
        command = state_machine_.update(
            data->time, ee_pos, observed_pose_ptr, idle_pose_reached);

        task_.onStateTransition(static_cast<int>(last_state_),static_cast<int>(current_state),data->time);
    }

    const int state_id = static_cast<int>(current_state);
    task_.ensureTrajectoryForState(command, state_id, data->time);
    Eigen::VectorXd q_cmd = task_.sampleJointTargets(data->time);
    maybeLogGraspDebug(data->time, ee_pose, command, observed_pose_ptr);

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
// 建立mujoco与pinocchio之间的关节索引表
void ClosedLoopRuntime::setupJointMaps(const mjModel* model) {
    mj_joints_.clear();
    // 从pinocchio中获取关节列表
    const auto& joints = task_.kinematics().joints();

    for (const auto& ji : joints) {
        MjJointMap map;
        map.name = ji.name;
        map.pin_q_adr = ji.pin_q_adr;
        map.pin_v_adr = ji.pin_v_adr;
        // 为每个pinocchio关节,在mujoco中查找对应的关节
        map.mj_joint_id = mj_name2id(model, mjOBJ_JOINT, ji.name.c_str());
        if (map.mj_joint_id < 0) {
            throw std::runtime_error("MuJoCo joint not found: " + ji.name);
        }
        map.mj_qpos_adr = model->jnt_qposadr[map.mj_joint_id];
        map.mj_dof_adr = model->jnt_dofadr[map.mj_joint_id];
        map.actuator_id = -1;
        // 将mujoco和pinocchio中的变量的地址全部保存到mj_joints中
        mj_joints_.push_back(map);
    }

    setupActuatorIds(model);
}
// 为每个关节找到在mujoco中对应的执行器ID,建立关节到执行器的联系
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
    // 通过pinocchio加载模型的广义位置坐标(nq)，和广义速度坐标(nv)
    const int nq = task_.kinematics().nq();
    const int nv = task_.kinematics().nv();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(nv);
    // 将mujoco当前的仿真状态同步到pinocchio
    for (const auto& map : mj_joints_) {
        q[map.pin_q_adr] = data->qpos[map.mj_qpos_adr];
        dq[map.pin_v_adr] = data->qvel[map.mj_dof_adr];
    }

    task_.setRobotState(q, dq);
}
// 根据状态机状态调整控制速度限制：
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
// 把 MuJoCo 已经算好的 `qfrc_bias` 直接写进 `qfrc_applied`。
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

void ClosedLoopRuntime::maybeLogGraspDebug(double now,
                                           const pinocchio::SE3& ee_pose,
                                           const GraspStateMachine::Command& command,
                                           const ObjectPose* observed_pose) {
    const auto state = state_machine_.state();
    if (state != GraspStateMachine::State::Close &&
        state != GraspStateMachine::State::Lift) {
        return;
    }
    if (last_debug_log_time_ >= 0.0 && (now - last_debug_log_time_) < 0.10) {
        return;
    }
    last_debug_log_time_ = now;

    const Eigen::Vector3d ee_pos = ee_pose.translation();
    std::cout << "[GraspDebug] t=" << now
              << " state=" << stateName(state)
              << " ee=(" << ee_pos.x() << "," << ee_pos.y() << "," << ee_pos.z() << ")"
              << " gripL=" << command.left_gripper_cmd
              << " gripR=" << command.right_gripper_cmd;

    if (observed_pose != nullptr) {
        const Eigen::Vector3d delta = observed_pose->position - ee_pos;
        std::cout << " block=("
                  << observed_pose->position.x() << ","
                  << observed_pose->position.y() << ","
                  << observed_pose->position.z() << ")"
                  << " block_minus_ee=("
                  << delta.x() << ","
                  << delta.y() << ","
                  << delta.z() << ")";
    } else {
        std::cout << " block=(missing)";
    }

    if (state == GraspStateMachine::State::Lift) {
        std::cout << " lift_target=("
                  << command.target_pos.x() << ","
                  << command.target_pos.y() << ","
                  << command.target_pos.z() << ")";
    }

    std::cout << std::endl;
}
