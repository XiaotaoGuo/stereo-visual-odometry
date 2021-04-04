//
// Created by guoxt on 19-8-24.
//

#ifndef STEORO_VISUAL_ODOMETRY_FEATURE_H
#define STEORO_VISUAL_ODOMETRY_FEATURE_H

#include "config.h"

class Frame;
class MapPoint;

class Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    weak_ptr<Frame> frame_;
    cv::KeyPoint position_;
    weak_ptr<MapPoint> map_point_;

    bool is_outlier = false;
    bool is_on_left_image_ = true;

    Feature();
    Feature(shared_ptr<Frame> frame, const cv::KeyPoint &kp);
};


#endif //STEORO_VISUAL_ODOMETRY_FEATURE_H
