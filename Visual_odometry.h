//
// Created by guoxt on 19-8-31.
//

#ifndef STEORO_VISUAL_ODOMETRY_VISUAL_ODOMETRY_H
#define STEORO_VISUAL_ODOMETRY_VISUAL_ODOMETRY_H

#include "config.h"

class Map;
class FrontEnd;
class BackEnd;
class Viewer;

class Visual_odometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Visual_odometry();

    bool init_camera();
    bool init();
    void start();
    bool forward();
    bool stop();

    private:
    shared_ptr<Camera> cam1_;
    shared_ptr<Camera> cam2_;
    shared_ptr<Map> map_;
    shared_ptr<FrontEnd> frontend_;
    shared_ptr<BackEnd> backend_;
    shared_ptr<Viewer> viewer_;
    int current_index;

    bool inited_;

};


#endif //STEORO_VISUAL_ODOMETRY_VISUAL_ODOMETRY_H
