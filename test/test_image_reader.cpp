#include <gtest/gtest.h>
#include <stereo_visual_odometry/io/data_reader.h>

TEST(KittiImageReaderTest, ReadSampleImages) {
    KittiImagesReader reader("../../data/sample/00", "image_0", "image_1", 0);

    int expect_num_images = 11;

    ASSERT_EQ(reader.GetNumImages(), expect_num_images);
    cv::Mat left_img;
    cv::Mat right_img;

    for (int i = 0; i < expect_num_images; ++i) {
        EXPECT_TRUE(reader.NextImages(left_img, right_img));
        cv::imshow("left", left_img);
        cv::imshow("right", right_img);
        cv::waitKey(100);
    }

    EXPECT_FALSE(reader.NextImages(left_img, right_img));
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}