//
// Created by guoxt on 19-8-24.
//

#ifndef STEORO_VISUAL_ODOMETRY_FRAME_H
#define STEORO_VISUAL_ODOMETRY_FRAME_H

#include "config.h"

class Feature;

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    unsigned long id_ = 0;
    unsigned long keyframe_id_ = 0;
    bool is_keyframe_ = false;
    double time_stamp_;
    mutex pose_mutex_;

    cv::Mat left_img_, right_img_;

    Sophus::SE3d pose_;

    vector<shared_ptr<Feature>> left_features_, right_features_;

    //constructor
    Frame();
    Frame(long id, double stamp, const Sophus::SE3d &pose, const cv::Mat &left, const cv::Mat &right);
    //get and set pose
    Sophus::SE3d Pose();
    void SetPose(const Sophus::SE3d &pose);
    //set to key frame
    void SetKeyFrame();

    static FramePtr CreateFrame();


};


#endif //STEORO_VISUAL_ODOMETRY_FRAME_H
