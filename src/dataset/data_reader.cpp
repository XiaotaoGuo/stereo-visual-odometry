#include "dataset/data_reader.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

namespace fs = boost::filesystem;

KittiDatasetReader::KittiDatasetReader(const std::string& data_root_path,
                                       int32_t sequence_id, int32_t index)
    : data_root_path_(data_root_path), index_(index) {
    sequence_path_ =
        data_root_path_ + (boost::format("%02d") % sequence_id).str();
    fs::path left_folder_path =
        fs::path(sequence_path_) / std::string("image_0");
    fs::path right_folder_path =
        fs::path(sequence_path_) / std::string("image_1");

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

bool KittiDatasetReader::GetStereoCameraConfig(
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix4d>>&
        vec_intrinsic_extrinsic) {
    std::ifstream fin(sequence_path_ + "/calib.txt");
    if (!fin) {
        std::cout << "cannot find " << sequence_path_ + "/calib.txt"
                  << std::endl;
        return false;
    }

    int idx = 0;
    while (idx < 2) {
        char camera_name[3];
        for (int k = 0; k < 3; ++k) {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }

        Eigen::Matrix3d K;
        Eigen::Vector3d t(0.0, 0.0, 0.0);
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        pose(0, 3) = projection_data[3];
        pose(1, 3) = projection_data[7];
        pose(2, 3) = projection_data[11];

        vec_intrinsic_extrinsic.push_back({K, pose});
        idx++;
    }
}

bool KittiDatasetReader::NextImages(cv::Mat& left_img, cv::Mat& right_img) {
    boost::format fmt("%06d.png");
    fs::path left_img_path = fs::path(sequence_path_) / std::string("image_0") /
                             (fmt % index_).str();
    fs::path right_img_path = fs::path(sequence_path_) /
                              std::string("image_1") / (fmt % index_).str();

    if (!fs::is_regular_file(left_img_path) ||
        !fs::is_regular_file(right_img_path)) {
        return false;
    }

    left_img = cv::imread(left_img_path.c_str(), cv::IMREAD_GRAYSCALE);
    right_img = cv::imread(right_img_path.c_str(), cv::IMREAD_GRAYSCALE);
    index_++;

    return true;
}