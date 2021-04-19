#ifndef DATA_READER_H_
#define DATA_READER_H_

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

class ImagesReader {
public:
    using Ptr = std::shared_ptr<ImagesReader>;
    using WkPtr = std::weak_ptr<ImagesReader>;

private:
    int8_t num_images_ = 0;  // number of images pairs

public:
    ImagesReader() {}
    virtual ~ImagesReader();

    virtual bool NextImages(cv::Mat& left_img, cv::Mat& right_img) = 0;

    void SetNumImages(int8_t num_images) { num_images_ = num_images; }
    int8_t GetNumImages() { return num_images_; }
};

class KittiImagesReader : public ImagesReader {
private:
    std::string data_root_path_;
    std::string left_folder_;
    std::string right_folder_;
    int8_t index_;

public:
    KittiImagesReader(const std::string& data_root_path,
                      const std::string& left_folder,
                      const std::string& right_folder, int8_t index = 0);

    virtual bool NextImages(cv::Mat& left_img, cv::Mat& right_img) override;
};

#endif