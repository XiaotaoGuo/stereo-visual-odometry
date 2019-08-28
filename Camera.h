#ifndef STEORO_VISUAL_ODOMETRY_CAMERA_H
#define STEORO_VISUAL_ODOMETRY_CAMERA_H

#include "config.h"

class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef shared_ptr<Camera> Ptr;

    Camera();
    Camera(Eigen::Vector4d intrinsics);

    //void WordToCam(Sophus::se3 )
};


#endif //STEORO_VISUAL_ODOMETRY_CAMERA_H
