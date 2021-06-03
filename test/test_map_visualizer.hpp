#include <gtest/gtest.h>
#include <chrono>
#include <thread>

#include "stereo_visual_odometry/visualization/map_visualizer.hpp"

TEST(MapVisualzerTest, SimpleTest) {
    using namespace stereo_visual_odometry;

    Map::Ptr map = std::make_shared<Map>();
    bool end_signal = false;
    MapVisualizer::Ptr visualizer =
        std::make_shared<MapVisualizer>(map, end_signal);

    std::thread render_loop;
    ASSERT_NO_THROW(render_loop =
                        std::thread([&visualizer]() { visualizer->Foward(); }));

    std::vector<MapPoint> sample_points{MapPoint(0, Eigen::Vector3d{1, 1, 0}),
                                        MapPoint(1, Eigen::Vector3d{-1, -1, 0}),
                                        MapPoint(2, Eigen::Vector3d{1, -1, 0}),
                                        MapPoint(3, Eigen::Vector3d{-1, 1, 0})};
    map->AddMapPoints(sample_points);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Sophus::SE3d pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    std::string image_folder = "../data/sample/00";
    cv::Mat left, right;

    for (int i = 0; i < 4; ++i) {
        t.x() += i;
        pose = Sophus::SE3d(R, t);
        std::string left_img_path =
            image_folder + "/image_0/00000" + std::to_string(i) + ".png";
        std::string right_img_path =
            image_folder + "/image_1/00000" + std::to_string(i) + ".png";
        left = cv::imread(left_img_path);
        right = cv::imread(right_img_path);
        cv::resize(left, left, cv::Size(0.5 * left.cols, 0.5 * left.rows));
        cv::resize(right, right, cv::Size(0.5 * right.cols, 0.5 * right.rows));
        Frame frame(i, left, right, pose);
        map->AddFrame(frame);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    end_signal = true;
    render_loop.join();
}