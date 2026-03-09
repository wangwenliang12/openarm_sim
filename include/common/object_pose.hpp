#ifndef OPENARM_OBJECT_POSE_HPP
#define OPENARM_OBJECT_POSE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

struct ObjectPose {
    int64_t utime = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
};

#endif
