#include "robot/cartesian_impedance_controller.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <iostream>

CartesianImpedanceController::CartesianImpedanceController(const std::string& urdf_path, const std::string& ee_link_name) {
    try {
        pinocchio::urdf::buildModel(urdf_path, model_);
        // data_存储的是机器人关节的数据结构
        data_ = pinocchio::Data(model_);
        
        if (model_.existFrame(ee_link_name)) {
            ee_frame_id_ = model_.getFrameId(ee_link_name);
            initialized_ = true;
            std::cout << "[Controller] Initialized for EE: " << ee_link_name << std::endl;
        } else {
            std::cerr << "[Controller] Error: Frame " << ee_link_name << " not found!" << std::endl;
            initialized_ = false;
        }

        // Default Gains
        Kp_ = Eigen::VectorXd::Zero(6);
        Kd_ = Eigen::VectorXd::Zero(6);
        Kp_ << 100, 100, 100, 10, 10, 10; // 100 N/m pos, 10 Nm/rad rot
        Kd_ << 10, 10, 10, 1, 1, 1;       // Damping
        // nv为机械臂的广义速度总数
        M_ = Eigen::MatrixXd::Zero(model_.nv,model_.nv);
        Minv_ = Eigen::MatrixXd::Zero(model_.nv,model_.nv);
        nle_ = Eigen::VectorXd::Zero(model_.nv);
        J_ = Eigen::MatrixXd::Zero(6,model_.nv);
        A_ = Eigen::MatrixXd::Zero(6,6);
    } catch (const std::exception& e) {
        std::cerr << "[Controller] Init failed: " << e.what() << std::endl;
    }
}

CartesianImpedanceController::~CartesianImpedanceController() {}

void CartesianImpedanceController::setGains(const Eigen::Vector3d& kp_pos, const Eigen::Vector3d& kp_rot, 
                                            const Eigen::Vector3d& kd_pos, const Eigen::Vector3d& kd_rot) {
    // head：提取Kp_数组中的前3个元素
    // tail：提取kp_数组中的最后3个元素
    Kp_.head(3) = kp_pos;
    Kp_.tail(3) = kp_rot;
    Kd_.head(3) = kd_pos;
    Kd_.tail(3) = kd_rot;
}
// SE3的数据结构可以理解成一个齐次旋转矩阵
pinocchio::SE3 CartesianImpedanceController::getCurrentPose(const Eigen::VectorXd& q) {
    if (!initialized_) return pinocchio::SE3::Identity();
    // 计算每个关节的位姿，将计算完的结果存在data.oMi中
    pinocchio::forwardKinematics(model_, data_, q);
    // 将关节，传感器，末端执行器等参考系转换到世界坐标系下，并且存入data_.oMf(oMf 代表每一个frame在世界坐标系下的位姿)
    pinocchio::updateFramePlacements(model_, data_);
    // 获取末端参考系在世界坐标系的位姿
    return data_.oMf[ee_frame_id_];
}

Eigen::VectorXd CartesianImpedanceController::computeTorque(const Eigen::VectorXd& q, 
                                                            const Eigen::VectorXd& dq, 
                                                            const Eigen::Vector3d& target_pos, 
                                                            const Eigen::Quaterniond& target_quat) {
    if (!initialized_) return Eigen::VectorXd::Zero(model_.nv);

    pinocchio::computeAllTerms(model_,data_,q,dq);
    pinocchio::updateFramePlacements(model_,data_);
    
    M_ = data_.M;
    M_.triangularView<Eigen::StrictlyLower>() = M_.transpose();
    nle_ = data_.nle;

    // 获取末端jacobi
    J_.setZero();
    pinocchio::getFrameJacobian(model_,data_,ee_frame_id_,pinocchio::LOCAL_WORLD_ALIGNED,J_);
    
    // 获取当前末端状态
    const pinocchio::SE3& current_pose = data_.oMf[ee_frame_id_];
    Eigen::Vector3d current_pos = current_pose.translation();
    Eigen::Quaterniond current_quat(current_pose.rotation());

    // 计算笛卡尔空间误差
    Eigen::Matrix<double,6,1> error;
    
    error.head(3) = target_pos - current_pos;

    Eigen::Quaterniond quat_err = target_quat * current_quat.inverse();
    quat_err.normalize();
    Eigen::AngleAxisd angle_axis(quat_err);
    error.tail(3) = angle_axis.axis()*angle_axis.angle();

    Eigen::Matrix<double,6,1> v_curr = J_ * dq;
    // getFrameAcceleration返回的是local坐标系下的加速度，需要转换回到world
    pinocchio::Motion a_drift_local = pinocchio::getFrameAcceleration(model_,data_,ee_frame_id_);
    pinocchio::Motion a_drift_world = data_.oMf[ee_frame_id_].act(a_drift_local);

    Eigen::Matrix<double,6,1> drift_acc = Eigen::Matrix<double,6,1> ::Zero();
    // 计算期望加速度
    Eigen::Matrix<double,6,1> x_dd_cmd = Kp_.cwiseProduct(error) - Kd_.cwiseProduct(v_curr);
    
    Eigen::Matrix<double,6,1> alpha = x_dd_cmd - drift_acc;

    Eigen::LLT<Eigen::MatrixXd> llt_M(M_);
    Minv_ = llt_M.solve(Eigen::MatrixXd::Identity(model_.nv,model_.nv));

    A_ = J_ * Minv_ * J_.transpose();

    double manipulability = sqrt(A_.determinant());
    double lambda = 0.0;
    double w_threshold = 0.01; // 阈值，根据实际机器人调整
    double lambda_max = 0.05;

    if (manipulability < w_threshold) {
        lambda = lambda_max * pow((1 - manipulability / w_threshold), 2);
    }

    Eigen::MatrixXd A_damped = A_ + (lambda*lambda) * Eigen::MatrixXd::Identity(6,6);

    Eigen::VectorXd y = A_damped.llt().solve(alpha);

    Eigen::VectorXd q_dd_cmd = Minv_ * J_.transpose() * y;

    Eigen::VectorXd tau  = M_ * q_dd_cmd + nle_;
    return tau;
}
