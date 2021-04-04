#ifndef STEORO_VISUAL_ODOMETRY_BACKEND_H
#define STEORO_VISUAL_ODOMETRY_BACKEND_H

#include <thread>
#include "config.h"
#include "Camera.h"
#include "Map.h"

//class Map;

class BackEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    BackEnd();

    void setCameras(CamPtr left, CamPtr right);
    void setMap(MapPtr map);
    void updateMap();
    void stop();

private:
    void BackEndLoop();

    void Optimize(FrameType& keyframes, LandmarkType& landmarks);
    MapPtr map_;
    thread backend_thread_;
    mutex data_mutex_;

    condition_variable map_update_;
    atomic<bool> backend_running_;

    CamPtr cam_left_ = nullptr, cam_right_ = nullptr;

};


#endif //STEORO_VISUAL_ODOMETRY_BACKEND_H
