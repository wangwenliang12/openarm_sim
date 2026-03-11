#ifndef OPENARM_ROBOT_KINEMATICS_HPP
#define OPENARM_ROBOT_KINEMATICS_HPP

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

struct JointInfo {
    std::string name;
    int pin_q_adr = -1;
    int pin_v_adr = -1;
    double lower_limit = 0.0;
    double upper_limit = 0.0;
    bool is_finger = false;
    enum class Group { LeftArm, RightArm, Other } group = Group::Other;
    bool home_fixed = false;
};

class RobotKinematics {
public:
    RobotKinematics(const std::string& urdf_path,
                    const std::string& ee_frame_name,
                    const std::vector<std::string>& right_arm_joints,
                    const std::vector<std::string>& left_arm_joints);

    void setRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);
    const Eigen::VectorXd& currentQ() const;

    pinocchio::SE3 getEndEffectorPose();

    Eigen::VectorXd solveIK(const Eigen::Vector3d& pos,
                            const Eigen::Quaterniond& quat,
                            const Eigen::VectorXd& seed) const;

    void setHomeQ(const Eigen::VectorXd& q);
    const Eigen::VectorXd& homeQ() const;

    const std::vector<JointInfo>& joints() const;
    int nq() const;
    int nv() const;

    const pinocchio::Model& model() const;
    Eigen::VectorXd extractRightArmQ(const Eigen::VectorXd& full_q) const;
    Eigen::VectorXd mergeRightArmQ(const Eigen::VectorXd& right_arm_q,
                                   const Eigen::VectorXd& base_q) const;
    std::vector<double> rightArmLowerBounds() const;
    std::vector<double> rightArmUpperBounds() const;
    int rightArmDof() const;
    Eigen::VectorXd solveRightArmIK(const Eigen::Vector3d& pos,
                                    const Eigen::Quaterniond& quat,
                                    const Eigen::VectorXd& seed_q) const;

private:
    pinocchio::Model pin_model_;
    mutable pinocchio::Data pin_data_;
    pinocchio::FrameIndex ee_frame_id_;
    std::vector<JointInfo> joints_;
    std::vector<int> right_arm_indices_;
    std::vector<int> left_arm_indices_;
    Eigen::VectorXd q_robot_, dq_robot_, home_q_;
};

#endif
