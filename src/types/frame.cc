#include "stereo_visual_odometry/types/frame.hpp"

namespace stereo_visual_odometry {

Frame::Frame(const cv::Mat& left_image, const cv::Mat& right_image,
             const Sophus::SE3d pose)
    : images_{left_image, right_image}, pose_(pose) {}
}  // namespace stereo_visual_odometry