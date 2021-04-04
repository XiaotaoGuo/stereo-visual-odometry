#include "Camera.h"

Camera::Camera() {
}

Camera::Camera(double fx, double fy, double cx, double cy, double baseline,
       const Sophus::SE3d &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
    pose_inv_ = pose_.inverse();
}

Sophus::SE3d Camera::pose() const { return pose_; }

Eigen::Matrix3d Camera::K() const {
    Eigen::Matrix3d k;
    k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    return k;
}

Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w) {
    return pose_ * T_c_w * p_w;
}

Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w) {
    return T_c_w.inverse() * pose_inv_ * p_c;
}

Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &p_c) {
    return Eigen::Vector2d(
            fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
            fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &p_p, double depth) {
    return Eigen::Vector3d(
            (p_p(0, 0) - cx_) * depth / fx_,
            (p_p(1, 0) - cy_) * depth / fy_,
            depth
    );
}

Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}

Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}