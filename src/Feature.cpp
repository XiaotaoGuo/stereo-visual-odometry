#include "Feature.h"

Feature::Feature() {}

Feature::Feature(shared_ptr <Frame> frame, const cv::KeyPoint &kp):frame_(frame), position_(kp) {}
