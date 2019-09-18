#ifndef STEORO_VISUAL_ODOMETRY_CAMERA_H
#define STEORO_VISUAL_ODOMETRY_CAMERA_H

#include "config.h"

class Camera {
public:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
            baseline_ = 0;  // Camera intrinsics
    Sophus::SE3d pose_;             // extrinsic, from stereo camera to single camera
    Sophus::SE3d pose_inv_;         // inverse of extrinsics

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline, const Sophus::SE3d &pose);

    Sophus::SE3d pose() const;

    // return intrinsic matrix
    Eigen::Matrix3d K() const;

    // coordinate transform: world, camera, pixel
    Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);

    Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w);

    Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);

    Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1);

    Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth = 1);

    Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);
};


#endif //STEORO_VISUAL_ODOMETRY_CAMERA_H
