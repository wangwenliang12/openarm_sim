#include "subscriber/lcm_pose_subscriber.hpp"

#include <openarm_lcm/ObjectPoseArray.hpp>

LcmPoseSubscriber::LcmPoseSubscriber(const std::string& channel)
    : channel_(channel), lcm_() {
    if (lcm_.good()) {
        lcm_.subscribe(channel_, &LcmPoseSubscriber::handleMessage, this);
    }
}

bool LcmPoseSubscriber::isReady() const {
    return lcm_.good();
}

void LcmPoseSubscriber::poll() {
    lcm_.handleTimeout(0);
}

bool LcmPoseSubscriber::getPose(const std::string& object_id, ObjectPose* out) const {
    const auto it = poses_.find(object_id);
    if (it == poses_.end()) {
        return false;
    }
    *out = it->second;
    return true;
}

void LcmPoseSubscriber::handleMessage(const lcm::ReceiveBuffer*,const std::string&,const openarm_lcm::ObjectPoseArray* msg) {
    for (const auto& pose : msg->poses) {
        ObjectPose cache;
        cache.utime = pose.utime;
        cache.position = Eigen::Vector3d(pose.position[0], pose.position[1], pose.position[2]);
        cache.quaternion = Eigen::Quaterniond(pose.quaternion[0], pose.quaternion[1], pose.quaternion[2], pose.quaternion[3]);
        cache.quaternion.normalize();
        poses_[pose.object_id] = cache;
    }
}
