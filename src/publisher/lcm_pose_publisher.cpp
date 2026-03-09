#include "publisher/lcm_pose_publisher.hpp"

#include <openarm_lcm/ObjectPoseArray.hpp>

#include <chrono>
#include <stdexcept>

namespace {
int64_t now_utime() {
    const auto now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    return now.time_since_epoch().count();
}
}

LcmPosePublisher::LcmPosePublisher(const std::string& channel)
    : channel_(channel), lcm_() {}

bool LcmPosePublisher::isReady() const {
    return lcm_.good();
}

void LcmPosePublisher::initialize(const mjModel* model) {
    red_block_body_id_ = mj_name2id(model, mjOBJ_BODY, "red_block");
    yellow_block_body_id_ = mj_name2id(model, mjOBJ_BODY, "yellow_block");
    blue_block_body_id_ = mj_name2id(model, mjOBJ_BODY, "blue_block");

    if (red_block_body_id_ < 0 || yellow_block_body_id_ < 0 || blue_block_body_id_ < 0) {
        throw std::runtime_error("Object body mapping failed in LcmPosePublisher");
    }
}

void LcmPosePublisher::publishTruthPoses(const mjModel* model, const mjData* data) {
    (void)model;
    openarm_lcm::ObjectPoseArray msg;
    msg.utime = now_utime();
    msg.num_poses = 3;
    msg.poses.resize(3);

    fillPoseMessage(data, red_block_body_id_, "red_block", &msg.poses[0]);
    fillPoseMessage(data, yellow_block_body_id_, "yellow_block", &msg.poses[1]);
    fillPoseMessage(data, blue_block_body_id_, "blue_block", &msg.poses[2]);

    lcm_.publish(channel_, &msg);
}

void LcmPosePublisher::fillPoseMessage(const mjData* data,
                                       int body_id,
                                       const std::string& object_id,
                                       openarm_lcm::ObjectPose* pose) const {
    pose->utime = now_utime();
    pose->object_id = object_id;
    pose->position[0] = data->xpos[3 * body_id + 0];
    pose->position[1] = data->xpos[3 * body_id + 1];
    pose->position[2] = data->xpos[3 * body_id + 2];
    pose->quaternion[0] = data->xquat[4 * body_id + 0];
    pose->quaternion[1] = data->xquat[4 * body_id + 1];
    pose->quaternion[2] = data->xquat[4 * body_id + 2];
    pose->quaternion[3] = data->xquat[4 * body_id + 3];
}
