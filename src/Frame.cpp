#include "Frame.h"
#include "Feature.h"

Frame::Frame() {}
Frame::Frame(long id, double stamp, const Sophus::SE3d &pose,
             const cv::Mat &left, const cv::Mat &right)
        :id_(id), time_stamp_(stamp), pose_(pose), left_img_(left), right_img_(right){
    is_keyframe_ = false;
    keyframe_id_ = 0;

}

Sophus::SE3d Frame::Pose(){
    unique_lock<mutex> lck(pose_mutex_);
    return pose_;
}

void Frame::SetPose(const Sophus::SE3d &pose) {
    unique_lock<mutex> lck(pose_mutex_);
    pose_ = pose;
}

FramePtr Frame::CreateFrame() {
    static long factord_id = 0;
    FramePtr new_frame(new Frame);
    new_frame->id_ = factord_id++;
    return new_frame;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}