#ifndef TYPES_MAP_H_
#define TYPES_MAP_H_

#include <memory>
#include <unordered_map>

#include "stereo_visual_odometry/types/frame.hpp"
#include "stereo_visual_odometry/types/map_point.hpp"

namespace stereo_visual_odometry {

class Map {
public:
    using Ptr = std::shared_ptr<Map>;

private:
    std::unordered_map<int32_t, Frame::Ptr> all_frames_;
    std::unordered_map<int32_t, MapPoint::Ptr> all_map_points_;
    Frame::Ptr latest_frame_;

public:
    Map() = default;

    ~Map() = default;

    bool AddFrame(const Frame& frame) {
        all_frames_[frame.Id()] = std::make_shared<Frame>(frame);
        latest_frame_ = all_frames_[frame.Id()];
        return true;
    }

    bool AddMapPoints(const std::vector<MapPoint>& map_points) {
        for (const MapPoint& point : map_points) {
            all_map_points_[point.Id()] = std::make_shared<MapPoint>(point);
        }

        return true;
    }

    Frame::Ptr GetLatestFrame() const { return latest_frame_; }

    std::unordered_map<int32_t, Frame::Ptr> GetAllFrames() const {
        return all_frames_;
    }

    std::unordered_map<int32_t, MapPoint::Ptr> GetAllMapPoints() const {
        return all_map_points_;
    }

private:
};
}  // namespace stereo_visual_odometry

#endif