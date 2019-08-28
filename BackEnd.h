//
// Created by guoxt on 19-8-26.
//

#ifndef STEORO_VISUAL_ODOMETRY_BACKEND_H
#define STEORO_VISUAL_ODOMETRY_BACKEND_H

#include "config.h"

class BackEnd;

class BackEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef shared_ptr<BackEnd> Ptr;

    BackEnd();

    void setCameras(shared_ptr<Camera> left, shared_ptr<Camera> right);
    void updateMap();
    void stop();

private:
    void BackendLoop();

    void Optimize(KeyframeType& keyframes, LandmarkType landmarks);
    shared_ptr<Map> map_;
    thread backend_thread_;
    mutex data_mutex_;

    condition_variable map_update_;
    atomic<bool> backend_running_;

    shared_ptr<Camera> cam_left_ = nullptr, cam_right_ = nullptr;

};


#endif //STEORO_VISUAL_ODOMETRY_BACKEND_H
