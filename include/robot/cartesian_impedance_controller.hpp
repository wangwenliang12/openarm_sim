#ifndef CARTESIAN_IMPEDANCE_CONTROLLER_HPP
#define CARTESIAN_IMPEDANCE_CONTROLLER_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <string>

class CartesianImpedanceController {
public:
    CartesianImpedanceController(const std::string& urdf_path, const std::string& ee_link_name);
    ~CartesianImpedanceController();

    /**
     * @brief Compute the task-space torque (Gravity/Coriolis must be added separately).
     * 
     * @param q Current joint positions.
     * @param dq Current joint velocities.
     * @param target_pos Desired Cartesian position (3D).
     * @param target_quat Desired Cartesian orientation (Quaternion w, x, y, z).
     * @return Eigen::VectorXd Joint torques (size nq).
     */
    Eigen::VectorXd computeTorque(const Eigen::VectorXd& q, 
                                  const Eigen::VectorXd& dq, 
                                  const Eigen::Vector3d& target_pos, 
                                  const Eigen::Quaterniond& target_quat);

    // Setters for gains
    void setGains(const Eigen::Vector3d& kp_pos, const Eigen::Vector3d& kp_rot, 
                  const Eigen::Vector3d& kd_pos, const Eigen::Vector3d& kd_rot);

    // Get current EE pose (useful for initializing target)
    pinocchio::SE3 getCurrentPose(const Eigen::VectorXd& q);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_id_;
    bool initialized_ = false;

    // Gains (6D: 3 Pos + 3 Rot)
    Eigen::VectorXd Kp_;
    Eigen::VectorXd Kd_;

    Eigen::MatrixXd M_;
    Eigen::MatrixXd Minv_;
    Eigen::VectorXd nle_;
    Eigen::MatrixXd J_;
    Eigen::MatrixXd A_;
};

#endif 
