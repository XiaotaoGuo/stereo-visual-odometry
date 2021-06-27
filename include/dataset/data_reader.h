
#ifndef DATA_READER_H_
#define DATA_READER_H_

#include <memory>
#include <string>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class DatasetReader {
public:
    using Ptr = std::shared_ptr<DatasetReader>;

private:
    int32_t num_images_ = 0;  ///< number of frames

public:
    DatasetReader() {}
    virtual ~DatasetReader(){};

    ///
    ///@brief Get the Stereo Camera Config
    ///
    ///@param vec_intrinsic_extrinsic a vector of intrinsic and extrinsic pair
    ///@return true if successdully load the config, otherwise return false
    ///
    virtual bool GetStereoCameraConfig(
        std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix4d>>
            &vec_intrinsic_extrinsic) = 0;

    ///
    ///@brief get next image pair
    ///
    ///@param left_img
    ///@param right_img
    ///@return true if success, otherwise return false
    ///
    virtual bool NextImages(cv::Mat &left_img, cv::Mat &right_img) = 0;

    void SetNumImages(int8_t num_images) { num_images_ = num_images; }
    int32_t GetNumImages() { return num_images_; }
};

class KittiDatasetReader : public DatasetReader {
private:
    std::string data_root_path_;
    std::string sequence_path_;
    int32_t index_;

public:
    ///
    ///@brief Construct a new Kitti Dataset Reader object
    ///
    ///@param data_root_path path to dataset (sequence folder)
    ///@param sequence_id id of sequence you want to test
    ///@param index index of first image you want to begin with, default with 0
    ///
    KittiDatasetReader(const std::string &data_root_path, int32_t sequence_id,
                       int32_t index = 0);

    ///
    ///@brief Get the Stereo Camera Config
    ///
    ///@param vec_intrinsic_extrinsic a vector of intrinsic and extrinsic pair
    ///@return true if successdully load the config, otherwise return false
    ///
    bool GetStereoCameraConfig(
        std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix4d>>
            &vec_intrinsic_extrinsic) override;

    ///
    ///@brief get next image pair
    ///
    ///@param left_img
    ///@param right_img
    ///@return true if success, otherwise return false
    ///
    bool NextImages(cv::Mat &left_img, cv::Mat &right_img) override;
};

#endif