#ifndef STEORO_VISUAL_ODOMETRY_MAP_H
#define STEORO_VISUAL_ODOMETRY_MAP_H

#include "config.h"
#include "Frame.h"
#include "MapPoint.h"

class MapPoint;
class Frame;

class Map {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef shared_ptr<Map> Ptr;
    //typedef unordered_map<unsigned long, shared_ptr<MapPoint>> LandmarkType;
    //typedef unordered_map<unsigned long, shared_ptr<Frame>> KeyframeType;

    Map();

    void InsertKeyFrame(shared_ptr<Frame> frame);
    void InsertMapPoint(shared_ptr<MapPoint> mappoint);

    LandmarkType GetAllMapPoints();
    KeyframeType GetAllKeyframes();
    LandmarkType GetActiveMapPoints();
    KeyframeType GetActiveKeyFrames();

    void CleanMap();


private:
    mutex data_mutex;

    LandmarkType landmarks_;
    LandmarkType active_landmarks_;
    KeyframeType keyframes_;
    KeyframeType active_keyframes_;

    shared_ptr<Frame> current_frame_ = nullptr;

    int num_active_keyframes_ = 7;

    void RemoveOldKeyframe();

};


#endif //STEORO_VISUAL_ODOMETRY_MAP_H
