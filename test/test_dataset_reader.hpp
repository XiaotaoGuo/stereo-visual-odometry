
#include <gtest/gtest.h>

#include "dataset/data_reader.h"

TEST(KittiReaderTest, ReadSampleImages) {
    KittiDatasetReader reader("../data/sample/", 00);

    int expect_num_images = 11;

    ASSERT_EQ(reader.GetNumImages(), expect_num_images);
    cv::Mat left_img;
    cv::Mat right_img;

    for (int i = 0; i < expect_num_images; ++i) {
        EXPECT_TRUE(reader.NextImages(left_img, right_img));
        cv::Mat frame;
        cv::hconcat(left_img, right_img, frame);
        cv::imshow("Frame", frame);
        cv::waitKey(100);
    }

    EXPECT_FALSE(reader.NextImages(left_img, right_img));
}