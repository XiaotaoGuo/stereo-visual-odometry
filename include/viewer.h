#ifndef STEORO_VISUAL_ODOMETRY_VIEWER_H
#define STEORO_VISUAL_ODOMETRY_VIEWER_H

#include <pangolin/pangolin.h>
#include <thread>

#include "Frame.h"
#include "Map.h"
#include "config.h"

class Map;
class Frame;
class MapPoint;

class Viewer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Viewer();

    void setMap(MapPtr map) { map_ = map; }

    void Close();

    void AddCurrentFrame(FramePtr current_frame);

    void UpdateMap();

private:
    void ThreadLoop();

    void DrawFrame(FramePtr frame, const float* color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    void UpdateTopDownTrajectory();

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage(const FramePtr frame);

    FramePtr current_frame_ = nullptr;
    FramePtr last_frame_ = nullptr;
    MapPtr map_ = nullptr;

    double window_width = (1024 + 620);
    double window_height = 768;
    double width_ = 1241 / 2;
    double height_ = 376 / 2;
    cv::Mat top_down_tajectory;

    thread viewer_thread_;
    bool viewer_running_ = true;

    FrameType active_keyframes_;
    LandmarkType active_landmarks_;
    FrameType allFrames_;
    bool map_updated_ = false;

    mutex viewer_data_mutex_;
};

#endif
