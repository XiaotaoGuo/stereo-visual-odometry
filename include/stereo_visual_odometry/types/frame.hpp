#ifndef TYPES_FRAMES_H_
#define TYPES_FRAMES_H_

#include <memory>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "stereo_visual_odometry/types/feature.hpp"

namespace stereo_visual_odometry {
class Frame {
public:
    using Ptr = std::shared_ptr<Frame>;
    using WkPtr = std::weak_ptr<Frame>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    int32_t id_ = 0;
    Sophus::SE3d pose_;
    std::pair<cv::Mat, cv::Mat> images_;  // left & right stereo images
    std::vector<Feature::Ptr> left_features_;
    std::vector<Feature::Ptr> right_features_;

public:
    Frame(const cv::Mat& left_image, const cv::Mat& right_image,
          const Sophus::SE3d pose);

    int32_t Id() const { return id_; }

private:
};
}  // namespace stereo_visual_odometry

#endif