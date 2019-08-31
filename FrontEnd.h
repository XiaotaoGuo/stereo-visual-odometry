//
// Created by guoxt on 19-8-25.
//

#ifndef STEORO_VISUAL_ODOMETRY_FRONTEND_H
#define STEORO_VISUAL_ODOMETRY_FRONTEND_H

#include "config.h"
#include "BackEnd.h"
#include <opencv2/features2d.hpp>


#define TRACKING_INIT 0
#define TRACKING_GOOD 1
#define TRACKING_BAD 2
#define LOST 3

class Camera;
class Frame;
class Map;
class Viewer;
class BackEnd;

class FrontEnd {
public:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<FrontEnd> Ptr;

    FrontEnd();

    /// 外部接口，添加一个帧并计算其定位结果
    bool AddFrame(shared_ptr<Frame> frame);

    /// Set函数
    void SetMap(shared_ptr<Map> map) { map_ = map; }

    void SetBackend(std::shared_ptr<BackEnd> backend) { backend_ = backend; }

    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    int GetStatus() const { return status_; }

    void SetCameras(shared_ptr<Camera> left, shared_ptr<Camera> right) {
        camera_left_ = left;
        camera_right_ = right;
    }

private:
    /**
     * Track in normal mode
     * @return true if success
     */
    bool Track();

    /**
     * Reset when lost
     * @return true if success
     */
    bool Reset();

    /**
     * Track with last frame
     * @return num of tracked points
     */
    int TrackLastFrame();

    /**
     * estimate current frame's pose
     * @return num of inliers
     */
    int EstimateCurrentPose();

    /**
     * set current frame as a keyframe and insert it into backend
     * @return true if success
     */
    bool InsertKeyframe();

    /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool StereoInit();

    /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
    int DetectFeatures();

    /**
     * Find the corresponding features in right image of current_frame_
     * @return num of features found
     */
    int FindFeaturesInRight();

    /**
     * Build the initial map with single image
     * @return true if succeed
     */
    bool BuildInitMap();

    /**
     * Triangulate the 2D points in current frame
     * @return num of triangulated points
     */
    int TriangulateNewPoints();

    /**
     * Set the features in keyframe as new observation of the map points
     */
    void SetObservationsForKeyFrame();

    // data
    int status_ = TRACKING_INIT;

    shared_ptr<Frame> current_frame_ = nullptr;  // 当前帧
    shared_ptr<Frame> last_frame_ = nullptr;     // 上一帧
    shared_ptr<Camera> camera_left_ = nullptr;   // 左侧相机
    shared_ptr<Camera> camera_right_ = nullptr;  // 右侧相机

    shared_ptr<Map> map_ = nullptr;
    std::shared_ptr<BackEnd> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    Sophus::SE3d relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
    bool triangulation(const vector<Sophus::SE3d>& poses, const vector<Eigen::Vector3d> points, Eigen::Vector3d& pWorlds);



};


#endif //STEORO_VISUAL_ODOMETRY_FRONTEND_H
