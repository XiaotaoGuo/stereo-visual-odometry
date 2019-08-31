//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "config.h"
#include "Frame.h"
#include "Map.h"

class Map;
class Frame;
class MapPoint;

class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(shared_ptr<Map> map) { map_ = map; }

    void Close();

    // 增加一个当前帧
    void AddCurrentFrame(shared_ptr<Frame> current_frame);

    // 更新地图
    void UpdateMap();

   private:
    void ThreadLoop();

    void DrawFrame(shared_ptr<Frame> frame, const float* color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    shared_ptr<Frame> current_frame_ = nullptr;
    shared_ptr<Map> map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, shared_ptr<Frame>> active_keyframes_;
    std::unordered_map<unsigned long, shared_ptr<MapPoint>> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};

#endif  // MYSLAM_VIEWER_H
