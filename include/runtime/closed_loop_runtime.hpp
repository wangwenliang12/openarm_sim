#ifndef OPENARM_CLOSED_LOOP_RUNTIME_HPP
#define OPENARM_CLOSED_LOOP_RUNTIME_HPP

#include "publisher/lcm_pose_publisher.hpp"
#include "state_machine/grasp_state_machine.hpp"
#include "subscriber/lcm_pose_subscriber.hpp"
#include "task/manipulate_task.hpp"

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

class ClosedLoopRuntime {
public:
    // 
    explicit ClosedLoopRuntime(const mjModel* model);
    // 先把 MuJoCo 当前状态同步到 `ManipulateTask`,用当前末端位姿初始化状态机,从状态机配置里加载命名姿态，再设置平滑命令初值
    void initialize(const mjModel* model, const mjData* data);
    // 每个控制周期执行一次
    void step(const mjModel* model, mjData* data);

private:
    // 记录同一个关节在不同系统中的映射例如mujoco和pinocchio,初始化时一次性建立映射
    struct MjJointMap {
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
    void syncRobotStateToTask(const mjData* data);
    ControlLimits controlLimitsForState() const;
    void applyRobotGravityCompensation(mjData* data) const;
    void writeArmPositionControls(mjData* data, const Eigen::VectorXd& q) const;
    void writeFingerControls(mjData* data, double left, double right) const;
    void writeHeadControls(mjData* data) const;
    void maybeLogGraspDebug(double now,
                            const pinocchio::SE3& ee_pose,
                            const GraspStateMachine::Command& command,
                            const ObjectPose* observed_pose);

    ManipulateTask task_;
    LcmPosePublisher publisher_;
    LcmPoseSubscriber subscriber_;
    GraspStateMachine state_machine_;

    std::vector<MjJointMap> mj_joints_;
    double control_timestep_ = 0.002;
    int head_actuator_1_ = -1;
    int head_actuator_2_ = -1;
    GraspStateMachine::State last_state_ = GraspStateMachine::State::Home;
    double last_debug_log_time_ = -1.0;
};

#endif
