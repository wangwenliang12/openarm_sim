#ifndef OPENARM_LCM_POSE_SUBSCRIBER_HPP
#define OPENARM_LCM_POSE_SUBSCRIBER_HPP

#include "common/object_pose.hpp"

#include <lcm/lcm-cpp.hpp>
#include <openarm_lcm/ObjectPoseArray.hpp>
#include <string>
#include <unordered_map>

class LcmPoseSubscriber {
public:
    explicit LcmPoseSubscriber(const std::string& channel);

    bool isReady() const;
    void poll();
    bool getPose(const std::string& object_id, ObjectPose* out) const;

private:
    void handleMessage(const lcm::ReceiveBuffer*,
                       const std::string&,
                       const openarm_lcm::ObjectPoseArray* msg);

    std::string channel_;
    lcm::LCM lcm_;
    std::unordered_map<std::string, ObjectPose> poses_;
};

#endif
