#ifndef TYPES_FEATURE_H_
#define TYPES_FEATURE_H_

#include <memory>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace stereo_visual_odometry {

class Frame;
class MapPoint;

class Feature {
public:
    using Ptr = std::shared_ptr<Feature>;
    using WkPtr = std::weak_ptr<Feature>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    cv::KeyPoint kp_pos_;

    // corrsponding frame of this current feature
    std::weak_ptr<Frame> frame_;
    // corresponding map point in global map
    std::weak_ptr<MapPoint> map_point_;

public:
    Feature();
};
}  // namespace stereo_visual_odometry

#endif