#include "stereo_visual_odometry/io/data_reader.h"

#include <iostream>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

namespace fs = boost::filesystem;

KittiImagesReader::KittiImagesReader(const std::string& data_root_path,
                                     const std::string& left_folder,
                                     const std::string& right_folder,
                                     int32_t index)
    : data_root_path_(data_root_path),
      left_folder_(left_folder),
      right_folder_(right_folder),
      index_(index) {
    fs::path left_folder_path = fs::path(data_root_path_) / left_folder_;
    fs::path right_folder_path = fs::path(data_root_path_) / right_folder_;

    if (!fs::exists(left_folder_path) || !fs::exists(right_folder_path)) {
        std::stringstream ss;
        ss << "Couldn't find: " << left_folder_path.c_str() << " or "
           << right_folder_path.c_str() << std::endl;
        throw std::runtime_error(ss.str());
    }

    boost::format fmt("%06d.png");
    int32_t curr_idx = index_;
    int32_t num_images = 0;

    fs::path left_img_path = left_folder_path / (fmt % curr_idx).str();
    fs::path right_img_path = right_folder_path / (fmt % curr_idx).str();
    while (fs::is_regular_file(left_img_path) &&
           fs::is_regular_file(right_img_path)) {
        num_images++;
        curr_idx++;
        left_img_path = left_folder_path / (fmt % curr_idx).str();
        right_img_path = right_folder_path / (fmt % curr_idx).str();
    }

    std::cout << "Total images: " << num_images << std::endl;
    SetNumImages(num_images);
}

bool KittiImagesReader::NextImages(cv::Mat& left_img, cv::Mat& right_img) {
    boost::format fmt("%06d.png");
    fs::path left_img_path =
        fs::path(data_root_path_) / left_folder_ / (fmt % index_).str();
    fs::path right_img_path =
        fs::path(data_root_path_) / right_folder_ / (fmt % index_).str();

    if (!fs::is_regular_file(left_img_path) ||
        !fs::is_regular_file(right_img_path)) {
        return false;
    }

    left_img = cv::imread(left_img_path.c_str(), cv::IMREAD_GRAYSCALE);
    right_img = cv::imread(right_img_path.c_str(), cv::IMREAD_GRAYSCALE);
    index_++;

    return true;
}