#include <gtest/gtest.h>
#include <chrono>
#include <thread>

#include "stereo_visual_odometry/visualization/map_visualizer.hpp"

TEST(MapVisualzerTest, SimpleTest) {
    using namespace stereo_visual_odometry;

    Map::Ptr map = std::make_shared<Map>();
    std::vector<MapPoint> sample_points{MapPoint(0, Eigen::Vector3d{0, 0, 0})};
    map->AddMapPoints(sample_points);

    bool end_signal = false;
    MapVisualizer::Ptr visualizer =
        std::make_shared<MapVisualizer>(map, end_signal);

    std::thread render_loop;
    ASSERT_NO_THROW(render_loop =
                        std::thread([&visualizer]() { visualizer->Foward(); }));

    std::this_thread::sleep_for(std::chrono::seconds(3));
    // end_signal = true;
    render_loop.join();
}