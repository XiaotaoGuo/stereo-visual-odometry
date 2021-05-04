#include "stereo_visual_odometry/types/frame.hpp"

namespace stereo_visual_odometry {

Frame::Frame(int32_t id, const cv::Mat& left_image, const cv::Mat& right_image,
             const Sophus::SE3d pose)
    : id_(id), images_{left_image, right_image}, pose_(pose) {}
}  // namespace stereo_visual_odometry