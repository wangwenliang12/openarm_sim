#ifndef OPENARM_LCM_POSE_PUBLISHER_HPP
#define OPENARM_LCM_POSE_PUBLISHER_HPP

#include <lcm/lcm-cpp.hpp>
#include <mujoco/mujoco.h>
#include <openarm_lcm/ObjectPose.hpp>
#include <string>

class LcmPosePublisher {
public:
    explicit LcmPosePublisher(const std::string& channel);

    bool isReady() const;
    void initialize(const mjModel* model);
    void publishTruthPoses(const mjModel* model, const mjData* data);

private:
    void fillPoseMessage(const mjData* data,int body_id,const std::string& object_id,openarm_lcm::ObjectPose* pose) const;

    std::string channel_;
    lcm::LCM lcm_;
    int red_block_body_id_ = -1;
    int yellow_block_body_id_ = -1;
    int blue_block_body_id_ = -1;
};

#endif
