#ifndef STEORO_VISUAL_ODOMETRY_MAP_H
#define STEORO_VISUAL_ODOMETRY_MAP_H

#include "Frame.h"
#include "MapPoint.h"
#include "config.h"

class MapPoint;
class Frame;

class Map {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    using Ptr = std::shared_ptr<Map>;

    Map();

    void insertKeyFrame(FramePtr frame);
    void insertMapPoint(MapPointPtr mappoint);
    void insertFrame(FramePtr frame);

    LandmarkType getAllMapPoints();
    FrameType getAllKeyframes();
    FrameType getAllFrames();
    LandmarkType getActiveMapPoints();
    FrameType getActiveKeyFrames();

    void cleanMap();

private:
    mutex data_mutex;

    LandmarkType landmarks_;
    LandmarkType active_landmarks_;
    FrameType keyframes_;
    FrameType active_keyframes_;
    FrameType frames_;

    FramePtr current_frame_ = nullptr;

    int num_active_keyframes_ = 7;

    void removeOldKeyframe();
};

#endif  // STEORO_VISUAL_ODOMETRY_MAP_H
