#ifndef TYPES_MAP_POINT_H_
#define TYPES_MAP_POINT_H_

#include <stdint.h>
#include <memory>

#include <Eigen/Core>

#include "stereo_visual_odometry/types/feature.hpp"

namespace stereo_visual_odometry {

class MapPoint {
public:
    using Ptr = std::shared_ptr<MapPoint>;
    using WkiPtr = std::weak_ptr<MapPoint>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    int32_t id_;
    Eigen::Vector3d pos_;  // global position

public:
    MapPoint(int32_t id, const Eigen::Vector3d& pos) : id_(id), pos_(pos) {}

    int32_t Id() const { return id_; }

    Eigen::Vector3d Pos() const { return pos_; }

private:
};

}  // namespace stereo_visual_odometry

#endif