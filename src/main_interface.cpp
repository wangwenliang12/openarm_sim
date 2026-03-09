#include <mujoco/mujoco.h>
#include <iostream>
#include <vector>
#include <memory> // 引入 smart pointers
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "robot/cartesian_impedance_controller.hpp"

// 全局控制器实例 (使用 unique_ptr 管理内存更安全)
std::unique_ptr<CartesianImpedanceController> controller;

// 轨迹参数
Eigen::Vector3d initial_pos;
Eigen::Quaterniond initial_quat;
bool cartesian_initialized = false;

// 关节空间 PD 参数 (用于 Phase 1)
const double Kp_joint = 50.0; // 稍微调大一点以抵抗重力误差
const double Kd_joint = 5.0;

extern "C" {

    void sim_init(const mjModel* m, mjData* d) {
        std::cout << "[C++] Initializing Cartesian Impedance Controller (Dynamics Version)..." << std::endl;

        std::string urdf_path = "model/urdf/robot/v10.bimanual.generated.urdf";
        std::string ee_link = "openarm_left_hand_tcp"; // 确保这个link名字正确

        // 【修改1】正确初始化全局变量，不要用 auto 声明局部变量
        controller = std::make_unique<CartesianImpedanceController>(urdf_path, ee_link);

        // 设置参数：因为现在有了动力学模型，Kp 可以设得大一些，系统依然稳定
        controller->setGains(
            Eigen::Vector3d(500, 500, 500),  // Pos Kp: 刚度更强
            Eigen::Vector3d(30, 30, 30),     // Rot Kp
            Eigen::Vector3d(20, 20, 20),     // Pos Kd: 阻尼
            Eigen::Vector3d(2, 2, 2)         // Rot Kd
        );
        
        cartesian_initialized = false;
        std::cout << "[C++] Phase 1: Moving to Home Pose (Joint Space)..." << std::endl;
    }

    void sim_step(const mjModel* m, mjData* d) {
        if (!controller) return;

        // 1. 读取状态 (Map MuJoCo data to Eigen)
        Eigen::Map<const Eigen::VectorXd> q(d->qpos, m->nq);
        Eigen::Map<const Eigen::VectorXd> dq(d->qvel, m->nv);
        
        // 读取 MuJoCo 计算的重力+科氏力 (bias force)
        // 注意：这对应 Pinocchio 的 nle (Non-Linear Effects)
        Eigen::Map<const Eigen::VectorXd> tau_bias(d->qfrc_bias, m->nv);

        Eigen::VectorXd tau_total = Eigen::VectorXd::Zero(m->nu);

        // ================= Phase 1: 关节空间 PD 控制 (前3秒) =================
        if (d->time < 3.0) {
            Eigen::VectorXd q_des = q; // 默认维持当前
            
            // 设定一个舒适的初始姿态
            // 假设这是 7 轴机械臂，确保索引没有越界
            if (m->nq >= 7) {
                q_des.setZero();
                q_des[1] = -0.3; 
                q_des[2] = -1.5; 
                q_des[4] = 1.5; 
            }

            // 关节力矩 = PD + 重力补偿
            // 在这里我们需要手动加 tau_bias，因为 PD 控制器不懂动力学
            tau_total = Kp_joint * (q_des - q) - Kd_joint * dq + tau_bias;
        }
        // ================= Phase 2: 笛卡尔阻抗控制 (3秒后) =================
        else {
            // 初始化轨迹起点
            if (!cartesian_initialized) {
                pinocchio::SE3 pose = controller->getCurrentPose(q);
                initial_pos = pose.translation();
                initial_quat = Eigen::Quaterniond(pose.rotation());
                cartesian_initialized = true;
                std::cout << "[C++] Phase 2: Impedance Control Active." << std::endl;
            }

            // 生成画圆轨迹
            double t = d->time - 3.0; 
            double radius = 0.15;
            double omega = 1.5; 

            Eigen::Vector3d target_pos = initial_pos;
            // 在 Y-Z 平面画圆
            target_pos.y() += radius * sin(omega * t);
            target_pos.z() += radius * (cos(omega * t) - 1.0); 

            // 【修改2】计算力矩
            // 新版 controller->computeTorque 内部已经包含了 M*q_dd + nle
            // 所以这里返回的已经是 全力矩 (Total Torque)
            Eigen::VectorXd tau_impedance = controller->computeTorque(q, dq, target_pos, initial_quat);
            
            // 【重要】这里直接赋值，绝对不要再加 tau_bias！
            tau_total = tau_impedance; 
            
            // 调试日志
            static int counter = 0;
            if (counter++ % 1000 == 0) {
                // 打印力矩大小，防止爆炸
                std::cout << "Time: " << t << " | Tau Max: " << tau_total.lpNorm<Eigen::Infinity>() << std::endl;
            }
        }

        // ================= 写入力矩到 MuJoCo =================
        // 假设 Actuator 是简单的力矩透传 (Torque motors)
        for (int i = 0; i < m->nu; ++i) {
            // 简单处理：假设 actuator_id == joint_id
            // 如果你的 xml 里 actuator 定义顺序和 joint 不一样，需要查表 m->actuator_trnid
            // 这里假设是直接映射的
            d->ctrl[i] = tau_total[i];
        }
    }
    
    void sim_close() {
        // unique_ptr 会自动释放，这里也可以显式 reset
        controller.reset();
    }
}